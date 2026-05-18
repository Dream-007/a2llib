/*
 * Copyright 2026
 * SPDX-License-Identifier: MIT
 *
 * XCP Master state machine and command builders.
 */

#include "xcp_master/xcp_master.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <utility>

#if defined(_WIN32)
#  include <windows.h>
#else
#  include <dlfcn.h>
#endif

namespace xcp_master {

namespace {

constexpr std::chrono::milliseconds kDefaultRecvTimeout{ 250 };

std::vector<uint8_t> MakePacket(XcpCommand cmd) {
  return std::vector<uint8_t>{ static_cast<uint8_t>(cmd) };
}

// ---------------------------------------------------------------------------
// Tiny dlopen / LoadLibrary abstraction used by Authenticate(library_path).
// Kept local so the public header doesn't need to leak <windows.h>.
// ---------------------------------------------------------------------------
#if defined(_WIN32)
using DllHandle = HMODULE;
inline DllHandle DllOpen(const char* path) { return ::LoadLibraryA(path); }
inline void* DllSym(DllHandle h, const char* name) {
  return reinterpret_cast<void*>(::GetProcAddress(h, name));
}
inline void DllClose(DllHandle h) { ::FreeLibrary(h); }
inline std::string DllError() {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "WinAPI error %lu",
                static_cast<unsigned long>(::GetLastError()));
  return buf;
}
#else
using DllHandle = void*;
inline DllHandle DllOpen(const char* path) {
  return ::dlopen(path, RTLD_NOW | RTLD_LOCAL);
}
inline void* DllSym(DllHandle h, const char* name) {
  return ::dlsym(h, name);
}
inline void DllClose(DllHandle h) { ::dlclose(h); }
inline std::string DllError() {
  const char* e = ::dlerror();
  return e ? std::string(e) : std::string("unknown dl error");
}
#endif

}  // namespace

// ===========================================================================
// XcpResult
// ===========================================================================
XcpResult XcpResult::Negative(uint8_t err) {
  XcpResult r;
  r.ok = false;
  r.error_code = err;
  r.error_name = std::string(XcpErrorName(err));
  r.error_description = std::string(XcpErrorDescription(err));
  return r;
}

XcpResult XcpResult::Failure(std::string description) {
  XcpResult r;
  r.ok = false;
  r.error_code = 0xFF;  // not a real XCP error, indicates transport failure
  r.error_name = "TRANSPORT_FAILURE";
  r.error_description = std::move(description);
  return r;
}

std::string XcpMaster::DescribeError(uint8_t code) {
  std::string out;
  out.append(XcpErrorName(code));
  out.append(": ");
  out.append(XcpErrorDescription(code));
  return out;
}

// ===========================================================================
// Lifecycle
// ===========================================================================
XcpMaster::XcpMaster(std::unique_ptr<XcpTransport> transport,
                     XcpMasterConfig cfg)
    : transport_(std::move(transport)), cfg_(std::move(cfg)) {}

XcpMaster::~XcpMaster() { Close(); }

bool XcpMaster::Open() {
  if (opened_) return true;
  if (!transport_) return false;
  if (!transport_->Open()) return false;
  opened_ = true;
  return true;
}

void XcpMaster::Close() {
  if (!opened_) return;
  if (transport_) transport_->Close();
  opened_ = false;
}

// ===========================================================================
// Low-level transaction
// ===========================================================================
bool XcpMaster::ReceiveResponse(std::vector<uint8_t>& out,
                                std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    std::vector<uint8_t> frame;
    const auto remaining = deadline - std::chrono::steady_clock::now();
    if (remaining <= std::chrono::milliseconds(0)) return false;
    if (!transport_->ReceivePacket(
            frame,
            std::chrono::duration_cast<std::chrono::milliseconds>(remaining))) {
      return false;
    }
    if (frame.empty()) continue;
    const uint8_t pid = frame[0];
    if (pid == kPidResp || pid == kPidErr || pid == kPidEv || pid == kPidServ) {
      out = std::move(frame);
      return true;
    }
    // Otherwise this is a DAQ DTO (pid < 0xFC).
    if (daq_cb_) {
      daq_cb_(pid, frame.data() + 1, frame.size() - 1);
    }
  }
  return false;
}

XcpResult XcpMaster::Transact(std::vector<uint8_t> packet,
                              std::chrono::milliseconds timeout) {
  if (!opened_ || !transport_) {
    return XcpResult::Failure("master not opened");
  }
  std::scoped_lock lk(io_mutex_);

  if (timeout <= std::chrono::milliseconds(0)) {
    timeout = cfg_.default_timeout > std::chrono::milliseconds(0)
                  ? cfg_.default_timeout
                  : kDefaultRecvTimeout;
  }

  if (!transport_->SendPacket(packet.data(), packet.size())) {
    return XcpResult::Failure("transport send failed");
  }

  std::vector<uint8_t> response;
  if (!ReceiveResponse(response, timeout)) {
    return XcpResult::Failure("timeout waiting for response");
  }

  const uint8_t pid = response[0];
  std::vector<uint8_t> payload(response.begin() + 1, response.end());
  if (pid == kPidResp) {
    return XcpResult::Positive(std::move(payload));
  }
  if (pid == kPidErr) {
    if (payload.empty()) {
      return XcpResult::Failure("malformed ERR (no error code)");
    }
    auto r = XcpResult::Negative(payload[0]);
    r.payload.assign(payload.begin() + 1, payload.end());
    return r;
  }
  // EV / SERV are async; surface them as failures so the caller may handle
  // them via a future event hook.
  std::string desc = pid == kPidEv ? "asynchronous EVENT instead of response"
                                   : "SERVICE_REQUEST instead of response";
  return XcpResult::Failure(std::move(desc));
}

XcpResult XcpMaster::SendCto(const std::vector<uint8_t>& packet,
                             std::chrono::milliseconds timeout) {
  return Transact(packet, timeout);
}

// ===========================================================================
// Standard commands
// ===========================================================================
XcpResult XcpMaster::Connect(uint8_t mode) {
  auto pkt = MakePacket(XcpCommand::CONNECT);
  pkt.push_back(mode);
  auto r = Transact(std::move(pkt), cfg_.connect_timeout);
  if (r) {
    // Cache the most-used values so subsequent commands can rely on them.
    auto decoded = DecodeConnect(r);
    if (decoded) {
      cfg_.max_cto = decoded->max_cto != 0 ? decoded->max_cto : cfg_.max_cto;
      cfg_.max_dto = decoded->max_dto != 0 ? decoded->max_dto : cfg_.max_dto;
      cfg_.byte_order = decoded->GetByteOrder();
      cfg_.granularity = decoded->GetAddressGranularity();
    }
  }
  return r;
}

XcpResult XcpMaster::Disconnect() { return Transact(MakePacket(XcpCommand::DISCONNECT), {}); }
XcpResult XcpMaster::GetStatus() { return Transact(MakePacket(XcpCommand::GET_STATUS), {}); }
XcpResult XcpMaster::Synch() { return Transact(MakePacket(XcpCommand::SYNCH), {}); }
XcpResult XcpMaster::GetCommModeInfo() { return Transact(MakePacket(XcpCommand::GET_COMM_MODE_INFO), {}); }

XcpResult XcpMaster::GetId(XcpIdType id_type) {
  auto pkt = MakePacket(XcpCommand::GET_ID);
  pkt.push_back(static_cast<uint8_t>(id_type));
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::SetRequest(uint8_t mode, uint16_t session_config_id) {
  auto pkt = MakePacket(XcpCommand::SET_REQUEST);
  pkt.push_back(mode);
  Pack16(pkt, session_config_id, cfg_.byte_order);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::GetSeed(uint8_t mode, uint8_t resource) {
  auto pkt = MakePacket(XcpCommand::GET_SEED);
  pkt.push_back(mode);
  pkt.push_back(resource);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::Unlock(uint8_t remaining_len, const uint8_t* key,
                            std::size_t len) {
  auto pkt = MakePacket(XcpCommand::UNLOCK);
  pkt.push_back(remaining_len);
  if (key && len > 0) {
    pkt.insert(pkt.end(), key, key + len);
  }
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::SetMta(uint32_t address, uint8_t addr_extension) {
  auto pkt = MakePacket(XcpCommand::SET_MTA);
  pkt.push_back(0);  // reserved
  pkt.push_back(0);  // reserved
  pkt.push_back(addr_extension);
  Pack32(pkt, address, cfg_.byte_order);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::Upload(uint8_t num_elements) {
  auto pkt = MakePacket(XcpCommand::UPLOAD);
  pkt.push_back(num_elements);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::ShortUpload(uint8_t num_elements, uint8_t addr_extension,
                                 uint32_t address) {
  auto pkt = MakePacket(XcpCommand::SHORT_UPLOAD);
  pkt.push_back(num_elements);
  pkt.push_back(0);  // reserved
  pkt.push_back(addr_extension);
  Pack32(pkt, address, cfg_.byte_order);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::BuildChecksum(uint32_t block_size) {
  auto pkt = MakePacket(XcpCommand::BUILD_CHECKSUM);
  pkt.push_back(0);
  pkt.push_back(0);
  pkt.push_back(0);
  Pack32(pkt, block_size, cfg_.byte_order);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::TransportLayerCmd(uint8_t sub_cmd, const uint8_t* params,
                                       std::size_t params_len) {
  auto pkt = MakePacket(XcpCommand::TRANSPORT_LAYER_CMD);
  pkt.push_back(sub_cmd);
  if (params && params_len > 0) {
    pkt.insert(pkt.end(), params, params + params_len);
  }
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::UserCmd(uint8_t sub_cmd, const uint8_t* params,
                             std::size_t params_len) {
  auto pkt = MakePacket(XcpCommand::USER_CMD);
  pkt.push_back(sub_cmd);
  if (params && params_len > 0) {
    pkt.insert(pkt.end(), params, params + params_len);
  }
  return Transact(std::move(pkt), {});
}

// --- Calibration -----------------------------------------------------------
XcpResult XcpMaster::Download(const uint8_t* data, std::size_t size) {
  auto pkt = MakePacket(XcpCommand::DOWNLOAD);
  pkt.push_back(static_cast<uint8_t>(size));
  if (data && size > 0) pkt.insert(pkt.end(), data, data + size);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::DownloadNext(const uint8_t* data, std::size_t size) {
  auto pkt = MakePacket(XcpCommand::DOWNLOAD_NEXT);
  pkt.push_back(static_cast<uint8_t>(size));
  if (data && size > 0) pkt.insert(pkt.end(), data, data + size);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::DownloadMax(const uint8_t* data, std::size_t size) {
  auto pkt = MakePacket(XcpCommand::DOWNLOAD_MAX);
  if (data && size > 0) pkt.insert(pkt.end(), data, data + size);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::ShortDownload(uint32_t address, uint8_t addr_extension,
                                   const uint8_t* data, std::size_t size) {
  auto pkt = MakePacket(XcpCommand::SHORT_DOWNLOAD);
  pkt.push_back(static_cast<uint8_t>(size));
  pkt.push_back(0);
  pkt.push_back(addr_extension);
  Pack32(pkt, address, cfg_.byte_order);
  if (data && size > 0) pkt.insert(pkt.end(), data, data + size);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::ModifyBits(uint8_t shift, uint16_t and_mask,
                                uint16_t xor_mask) {
  auto pkt = MakePacket(XcpCommand::MODIFY_BITS);
  pkt.push_back(shift);
  Pack16(pkt, and_mask, cfg_.byte_order);
  Pack16(pkt, xor_mask, cfg_.byte_order);
  return Transact(std::move(pkt), {});
}

// --- Page switching --------------------------------------------------------
XcpResult XcpMaster::SetCalPage(uint8_t mode, uint8_t segment, uint8_t page) {
  auto pkt = MakePacket(XcpCommand::SET_CAL_PAGE);
  pkt.push_back(mode);
  pkt.push_back(segment);
  pkt.push_back(page);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::GetCalPage(uint8_t access_mode, uint8_t segment) {
  auto pkt = MakePacket(XcpCommand::GET_CAL_PAGE);
  pkt.push_back(access_mode);
  pkt.push_back(segment);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::GetPagProcessorInfo() {
  return Transact(MakePacket(XcpCommand::GET_PAG_PROCESSOR_INFO), {});
}

XcpResult XcpMaster::GetSegmentInfo(uint8_t mode, uint8_t segment,
                                    uint8_t segment_info,
                                    uint8_t mapping_index) {
  auto pkt = MakePacket(XcpCommand::GET_SEGMENT_INFO);
  pkt.push_back(mode);
  pkt.push_back(segment);
  pkt.push_back(segment_info);
  pkt.push_back(mapping_index);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::GetPageInfo(uint8_t segment, uint8_t page) {
  auto pkt = MakePacket(XcpCommand::GET_PAGE_INFO);
  pkt.push_back(0);
  pkt.push_back(segment);
  pkt.push_back(page);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::SetSegmentMode(uint8_t mode, uint8_t segment) {
  auto pkt = MakePacket(XcpCommand::SET_SEGMENT_MODE);
  pkt.push_back(mode);
  pkt.push_back(segment);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::GetSegmentMode(uint8_t segment) {
  auto pkt = MakePacket(XcpCommand::GET_SEGMENT_MODE);
  pkt.push_back(0);
  pkt.push_back(segment);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::CopyCalPage(uint8_t src_segment, uint8_t src_page,
                                 uint8_t dst_segment, uint8_t dst_page) {
  auto pkt = MakePacket(XcpCommand::COPY_CAL_PAGE);
  pkt.push_back(src_segment);
  pkt.push_back(src_page);
  pkt.push_back(dst_segment);
  pkt.push_back(dst_page);
  return Transact(std::move(pkt), {});
}

// --- DAQ -------------------------------------------------------------------
XcpResult XcpMaster::ClearDaqList(uint16_t daq_list) {
  auto pkt = MakePacket(XcpCommand::CLEAR_DAQ_LIST);
  pkt.push_back(0);
  Pack16(pkt, daq_list, cfg_.byte_order);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::SetDaqPtr(uint16_t daq_list, uint8_t odt,
                               uint8_t odt_entry) {
  auto pkt = MakePacket(XcpCommand::SET_DAQ_PTR);
  pkt.push_back(0);
  Pack16(pkt, daq_list, cfg_.byte_order);
  pkt.push_back(odt);
  pkt.push_back(odt_entry);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::WriteDaq(uint8_t bit_offset, uint8_t size,
                              uint8_t addr_extension, uint32_t address) {
  auto pkt = MakePacket(XcpCommand::WRITE_DAQ);
  pkt.push_back(bit_offset);
  pkt.push_back(size);
  pkt.push_back(addr_extension);
  Pack32(pkt, address, cfg_.byte_order);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::SetDaqListMode(uint8_t mode, uint16_t daq_list,
                                    uint16_t event_channel, uint8_t prescaler,
                                    uint8_t priority) {
  auto pkt = MakePacket(XcpCommand::SET_DAQ_LIST_MODE);
  pkt.push_back(mode);
  Pack16(pkt, daq_list, cfg_.byte_order);
  Pack16(pkt, event_channel, cfg_.byte_order);
  pkt.push_back(prescaler);
  pkt.push_back(priority);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::GetDaqListMode(uint16_t daq_list) {
  auto pkt = MakePacket(XcpCommand::GET_DAQ_LIST_MODE);
  pkt.push_back(0);
  Pack16(pkt, daq_list, cfg_.byte_order);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::StartStopDaqList(XcpStartStopMode mode, uint16_t daq_list) {
  auto pkt = MakePacket(XcpCommand::START_STOP_DAQ_LIST);
  pkt.push_back(static_cast<uint8_t>(mode));
  Pack16(pkt, daq_list, cfg_.byte_order);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::StartStopSynch(XcpStartStopSynch mode) {
  auto pkt = MakePacket(XcpCommand::START_STOP_SYNCH);
  pkt.push_back(static_cast<uint8_t>(mode));
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::GetDaqClock() { return Transact(MakePacket(XcpCommand::GET_DAQ_CLOCK), {}); }
XcpResult XcpMaster::ReadDaq() { return Transact(MakePacket(XcpCommand::READ_DAQ), {}); }
XcpResult XcpMaster::GetDaqProcessorInfo() { return Transact(MakePacket(XcpCommand::GET_DAQ_PROCESSOR_INFO), {}); }
XcpResult XcpMaster::GetDaqResolutionInfo() { return Transact(MakePacket(XcpCommand::GET_DAQ_RESOLUTION_INFO), {}); }

XcpResult XcpMaster::GetDaqListInfo(uint16_t daq_list) {
  auto pkt = MakePacket(XcpCommand::GET_DAQ_LIST_INFO);
  pkt.push_back(0);
  Pack16(pkt, daq_list, cfg_.byte_order);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::GetDaqEventInfo(uint16_t event_channel) {
  auto pkt = MakePacket(XcpCommand::GET_DAQ_EVENT_INFO);
  pkt.push_back(0);
  Pack16(pkt, event_channel, cfg_.byte_order);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::FreeDaq() { return Transact(MakePacket(XcpCommand::FREE_DAQ), {}); }

XcpResult XcpMaster::AllocDaq(uint16_t daq_count) {
  auto pkt = MakePacket(XcpCommand::ALLOC_DAQ);
  pkt.push_back(0);
  Pack16(pkt, daq_count, cfg_.byte_order);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::AllocOdt(uint16_t daq_list, uint8_t odt_count) {
  auto pkt = MakePacket(XcpCommand::ALLOC_ODT);
  pkt.push_back(0);
  Pack16(pkt, daq_list, cfg_.byte_order);
  pkt.push_back(odt_count);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::AllocOdtEntry(uint16_t daq_list, uint8_t odt,
                                   uint8_t entry_count) {
  auto pkt = MakePacket(XcpCommand::ALLOC_ODT_ENTRY);
  pkt.push_back(0);
  Pack16(pkt, daq_list, cfg_.byte_order);
  pkt.push_back(odt);
  pkt.push_back(entry_count);
  return Transact(std::move(pkt), {});
}

// --- Flash programming -----------------------------------------------------
XcpResult XcpMaster::ProgramStart() {
  return Transact(MakePacket(XcpCommand::PROGRAM_START), {});
}

XcpResult XcpMaster::ProgramClear(uint8_t mode, uint32_t clear_range) {
  auto pkt = MakePacket(XcpCommand::PROGRAM_CLEAR);
  pkt.push_back(mode);
  pkt.push_back(0);
  pkt.push_back(0);
  Pack32(pkt, clear_range, cfg_.byte_order);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::Program(const uint8_t* data, std::size_t size) {
  auto pkt = MakePacket(XcpCommand::PROGRAM);
  pkt.push_back(static_cast<uint8_t>(size));
  if (data && size > 0) pkt.insert(pkt.end(), data, data + size);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::ProgramReset() {
  return Transact(MakePacket(XcpCommand::PROGRAM_RESET), {});
}

XcpResult XcpMaster::GetPgmProcessorInfo() {
  return Transact(MakePacket(XcpCommand::GET_PGM_PROCESSOR_INFO), {});
}

XcpResult XcpMaster::GetSectorInfo(uint8_t mode, uint8_t sector) {
  auto pkt = MakePacket(XcpCommand::GET_SECTOR_INFO);
  pkt.push_back(mode);
  pkt.push_back(sector);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::ProgramPrepare(uint16_t code_size) {
  auto pkt = MakePacket(XcpCommand::PROGRAM_PREPARE);
  pkt.push_back(0);
  Pack16(pkt, code_size, cfg_.byte_order);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::ProgramFormat(uint8_t compression, uint8_t encryption,
                                   uint8_t prog_method, uint8_t access) {
  auto pkt = MakePacket(XcpCommand::PROGRAM_FORMAT);
  pkt.push_back(compression);
  pkt.push_back(encryption);
  pkt.push_back(prog_method);
  pkt.push_back(access);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::ProgramNext(const uint8_t* data, std::size_t size) {
  auto pkt = MakePacket(XcpCommand::PROGRAM_NEXT);
  pkt.push_back(static_cast<uint8_t>(size));
  if (data && size > 0) pkt.insert(pkt.end(), data, data + size);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::ProgramMax(const uint8_t* data, std::size_t size) {
  auto pkt = MakePacket(XcpCommand::PROGRAM_MAX);
  if (data && size > 0) pkt.insert(pkt.end(), data, data + size);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::ProgramVerify(uint8_t mode, uint16_t type, uint32_t value) {
  auto pkt = MakePacket(XcpCommand::PROGRAM_VERIFY);
  pkt.push_back(mode);
  Pack16(pkt, type, cfg_.byte_order);
  Pack32(pkt, value, cfg_.byte_order);
  return Transact(std::move(pkt), {});
}

// --- 1.3+ commands ---------------------------------------------------------
XcpResult XcpMaster::WriteDaqMultiple(
    const std::vector<uint8_t>& packed_entries) {
  auto pkt = MakePacket(XcpCommand::WRITE_DAQ_MULTIPLE);
  pkt.insert(pkt.end(), packed_entries.begin(), packed_entries.end());
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::TimeCorrelationProperties(
    uint8_t set_properties, uint8_t get_properties_request,
    uint16_t cluster_id) {
  auto pkt = MakePacket(XcpCommand::TIME_CORRELATION_PROPERTIES);
  pkt.push_back(set_properties);
  pkt.push_back(get_properties_request);
  pkt.push_back(0);
  Pack16(pkt, cluster_id, cfg_.byte_order);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::DtoCtrProperties(uint8_t modifier, uint16_t event_channel,
                                      uint16_t related_event_channel,
                                      uint8_t mode) {
  auto pkt = MakePacket(XcpCommand::DTO_CTR_PROPERTIES);
  pkt.push_back(modifier);
  Pack16(pkt, event_channel, cfg_.byte_order);
  Pack16(pkt, related_event_channel, cfg_.byte_order);
  pkt.push_back(mode);
  return Transact(std::move(pkt), {});
}

// --- Level 1 escape --------------------------------------------------------
XcpResult XcpMaster::GetVersion() {
  auto pkt = MakePacket(XcpCommand::STD_CMD_LVL1);
  pkt.push_back(static_cast<uint8_t>(XcpCmdLevel1::GET_VERSION));
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::SetDaqPackedMode(uint16_t daq_list, uint8_t mode,
                                      uint16_t timestamp_mode,
                                      uint16_t sample_count) {
  auto pkt = MakePacket(XcpCommand::STD_CMD_LVL1);
  pkt.push_back(static_cast<uint8_t>(XcpCmdLevel1::SET_DAQ_PACKED_MODE));
  Pack16(pkt, daq_list, cfg_.byte_order);
  pkt.push_back(mode);
  Pack16(pkt, timestamp_mode, cfg_.byte_order);
  Pack16(pkt, sample_count, cfg_.byte_order);
  return Transact(std::move(pkt), {});
}

XcpResult XcpMaster::GetDaqPackedMode(uint16_t daq_list) {
  auto pkt = MakePacket(XcpCommand::STD_CMD_LVL1);
  pkt.push_back(static_cast<uint8_t>(XcpCmdLevel1::GET_DAQ_PACKED_MODE));
  Pack16(pkt, daq_list, cfg_.byte_order);
  return Transact(std::move(pkt), {});
}

// ===========================================================================
// Decoders
// ===========================================================================
std::optional<ConnectResponse> XcpMaster::DecodeConnect(const XcpResult& r) {
  if (!r || r.payload.size() < 7) return std::nullopt;
  ConnectResponse out;
  out.resource = r.payload[0];
  out.comm_mode_basic = r.payload[1];
  out.max_cto = r.payload[2];
  // MAX_DTO byte order follows CMB_BYTE_ORDER, which is reported in the same
  // packet.  Decode using the freshly reported byte order.
  const ByteOrder bo = (out.comm_mode_basic & CMB_BYTE_ORDER) ? ByteOrder::BIG
                                                              : ByteOrder::LITTLE;
  out.max_dto = Unpack16(&r.payload[3], bo);
  out.protocol_layer_version = r.payload[5];
  out.transport_layer_version = r.payload[6];
  return out;
}

std::optional<GetStatusResponse> XcpMaster::DecodeGetStatus(const XcpResult& r) {
  if (!r || r.payload.size() < 5) return std::nullopt;
  GetStatusResponse out;
  out.current_session_status = r.payload[0];
  out.current_resource_protection = r.payload[1];
  out.reserved = r.payload[2];
  out.session_config_id = Unpack16(&r.payload[3], cfg_.byte_order);
  return out;
}

std::optional<GetCommModeInfoResponse> XcpMaster::DecodeGetCommModeInfo(
    const XcpResult& r) {
  if (!r || r.payload.size() < 7) return std::nullopt;
  GetCommModeInfoResponse out;
  // payload[0] reserved
  out.comm_mode_optional = r.payload[1];
  // payload[2] reserved
  out.max_bs = r.payload[3];
  out.min_st = r.payload[4];
  out.queue_size = r.payload[5];
  out.xcp_driver_version = r.payload[6];
  return out;
}

std::optional<GetDaqProcessorInfoResponse> XcpMaster::DecodeGetDaqProcessorInfo(
    const XcpResult& r) {
  if (!r || r.payload.size() < 7) return std::nullopt;
  GetDaqProcessorInfoResponse out;
  out.daq_properties = r.payload[0];
  out.max_daq = Unpack16(&r.payload[1], cfg_.byte_order);
  out.max_event_channel = Unpack16(&r.payload[3], cfg_.byte_order);
  out.min_daq = r.payload[5];
  out.daq_key_byte = r.payload[6];
  return out;
}

std::optional<GetDaqResolutionInfoResponse>
XcpMaster::DecodeGetDaqResolutionInfo(const XcpResult& r) {
  if (!r || r.payload.size() < 7) return std::nullopt;
  GetDaqResolutionInfoResponse out;
  out.granularity_odt_entry_size_daq = r.payload[0];
  out.max_odt_entry_size_daq = r.payload[1];
  out.granularity_odt_entry_size_stim = r.payload[2];
  out.max_odt_entry_size_stim = r.payload[3];
  out.timestamp_mode = r.payload[4];
  out.timestamp_ticks = Unpack16(&r.payload[5], cfg_.byte_order);
  return out;
}

// ===========================================================================
// Block-mode helpers
// ===========================================================================
XcpResult XcpMaster::DownloadBlock(uint32_t address, uint8_t addr_extension,
                                   const uint8_t* data, std::size_t size) {
  if (data == nullptr || size == 0) {
    return XcpResult::Failure("empty download buffer");
  }

  auto mta = SetMta(address, addr_extension);
  if (!mta) return mta;

  // CTO byte 0 = PID, byte 1 = count, payload starts at byte 2.
  if (cfg_.max_cto < 3) {
    return XcpResult::Failure("max_cto too small for DOWNLOAD");
  }
  const std::size_t payload_cap = cfg_.max_cto - 2;

  // First frame: DOWNLOAD carries the *total* number of elements.
  const std::size_t first_chunk = std::min<std::size_t>(payload_cap, size);
  {
    // Manually build the first packet so byte 1 is the total size, not the
    // current chunk size (DOWNLOAD allows count > payload-in-this-frame).
    std::vector<uint8_t> pkt = { static_cast<uint8_t>(XcpCommand::DOWNLOAD),
                                 static_cast<uint8_t>(size) };
    pkt.insert(pkt.end(), data, data + first_chunk);
    auto r = Transact(std::move(pkt), {});
    if (!r) return r;
  }

  std::size_t offset = first_chunk;
  std::size_t remaining = size - offset;
  while (remaining > 0) {
    const std::size_t chunk = std::min<std::size_t>(payload_cap, remaining);
    std::vector<uint8_t> pkt = { static_cast<uint8_t>(XcpCommand::DOWNLOAD_NEXT),
                                 static_cast<uint8_t>(remaining) };
    pkt.insert(pkt.end(), data + offset, data + offset + chunk);
    auto r = Transact(std::move(pkt), {});
    if (!r) return r;
    offset += chunk;
    remaining -= chunk;
  }
  XcpResult done;
  done.ok = true;
  return done;
}

XcpResult XcpMaster::UploadBlock(uint32_t address, uint8_t addr_extension,
                                 std::size_t size, std::vector<uint8_t>& out) {
  out.clear();
  out.reserve(size);
  auto mta = SetMta(address, addr_extension);
  if (!mta) return mta;
  // MAX_CTO byte 0 = PID, payload up to MAX_CTO - 1.
  if (cfg_.max_cto < 2) {
    return XcpResult::Failure("max_cto too small for UPLOAD");
  }
  while (out.size() < size) {
    const std::size_t chunk = std::min<std::size_t>(cfg_.max_cto - 1,
                                                    size - out.size());
    auto r = Upload(static_cast<uint8_t>(chunk));
    if (!r) return r;
    out.insert(out.end(), r.payload.begin(), r.payload.end());
  }
  if (out.size() > size) out.resize(size);
  XcpResult done;
  done.ok = true;
  return done;
}

// ===========================================================================
// Seed & Key helpers
// ===========================================================================
XcpResult XcpMaster::GetSeedComplete(uint8_t resource,
                                     std::vector<uint8_t>& seed_out) {
  seed_out.clear();
  // GET_SEED response is FF <LEN> <seed bytes...>, so MAX_CTO must allow at
  // least one seed byte: PID + LEN + 1 = 3.
  if (cfg_.max_cto < 3) {
    return XcpResult::Failure("max_cto too small for GET_SEED");
  }
  uint8_t mode = 0;
  while (true) {
    auto r = GetSeed(mode, mode == 0 ? resource : 0);
    if (!r) return r;
    if (r.payload.empty()) {
      return XcpResult::Failure("GET_SEED response missing length byte");
    }
    const uint8_t remaining = r.payload[0];
    const std::size_t chunk_size = r.payload.size() - 1;
    if (mode == 0 && remaining == 0) {
      // Spec: LENGTH=0 on the first request means the resource is already
      // unlocked; the slave sends no seed bytes.
      return r;
    }
    if (chunk_size == 0) {
      return XcpResult::Failure("GET_SEED returned no seed bytes");
    }
    seed_out.insert(seed_out.end(), r.payload.begin() + 1, r.payload.end());
    if (chunk_size >= remaining) break;
    mode = 1;
  }
  XcpResult done;
  done.ok = true;
  return done;
}

XcpResult XcpMaster::SendKey(const uint8_t* key, std::size_t key_len) {
  if (key == nullptr || key_len == 0) {
    return XcpResult::Failure("empty key");
  }
  // UNLOCK REMAINING is a single byte — the spec caps key length at 255.
  if (key_len > 0xFF) {
    return XcpResult::Failure(
        "key length exceeds 255 bytes (UNLOCK REMAINING is 8-bit)");
  }
  // UNLOCK frame is F7 <REMAINING> <key bytes...>, so MAX_CTO must allow at
  // least one key byte.
  if (cfg_.max_cto < 3) {
    return XcpResult::Failure("max_cto too small for UNLOCK");
  }
  const std::size_t chunk_cap = cfg_.max_cto - 2;
  std::size_t offset = 0;
  XcpResult last;
  while (offset < key_len) {
    const std::size_t remaining = key_len - offset;
    const std::size_t chunk = std::min(remaining, chunk_cap);
    last = Unlock(static_cast<uint8_t>(remaining), key + offset, chunk);
    if (!last) return last;
    offset += chunk;
  }
  return last;
}

XcpResult XcpMaster::Authenticate(uint8_t resource,
                                  XcpComputeKeyFromSeedFn compute) {
  if (compute == nullptr) {
    return XcpResult::Failure("Authenticate: null compute function");
  }

  std::vector<uint8_t> seed;
  auto get = GetSeedComplete(resource, seed);
  if (!get) return get;
  if (seed.empty()) {
    // Resource was already unlocked — surface the GET_SEED response unchanged.
    return get;
  }
  if (seed.size() > 0xFF) {
    return XcpResult::Failure("seed length exceeds 255 bytes");
  }

  // Vector/ETAS convention: caller pre-fills key_length with the buffer
  // capacity; callee overwrites it with the actual key length on success.
  std::vector<uint8_t> key(0xFF, 0);
  uint8_t key_len = static_cast<uint8_t>(key.size());
  const uint32_t rc =
      compute(resource, seed.data(), static_cast<uint8_t>(seed.size()),
              key.data(), &key_len);
  if (rc != 0) {
    char buf[64];
    std::snprintf(buf, sizeof(buf),
                  "XCP_ComputeKeyFromSeed returned 0x%08X",
                  static_cast<unsigned>(rc));
    return XcpResult::Failure(buf);
  }
  if (key_len == 0) {
    return XcpResult::Failure("XCP_ComputeKeyFromSeed returned zero-length key");
  }
  return SendKey(key.data(), key_len);
}

XcpResult XcpMaster::Authenticate(uint8_t resource,
                                  const std::string& library_path) {
  if (library_path.empty()) {
    return XcpResult::Failure("Authenticate: empty library path");
  }
  DllHandle handle = DllOpen(library_path.c_str());
  if (handle == nullptr) {
    return XcpResult::Failure("failed to load '" + library_path +
                              "': " + DllError());
  }
  auto fn = reinterpret_cast<XcpComputeKeyFromSeedFn>(
      DllSym(handle, kSeedKeyFunctionName));
  if (fn == nullptr) {
    std::string err = std::string("symbol '") + kSeedKeyFunctionName +
                      "' not found in '" + library_path + "': " + DllError();
    DllClose(handle);
    return XcpResult::Failure(std::move(err));
  }
  auto result = Authenticate(resource, fn);
  DllClose(handle);
  return result;
}

}  // namespace xcp_master
