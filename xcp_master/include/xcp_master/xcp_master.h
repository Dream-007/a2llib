/*
 * Copyright 2026
 * SPDX-License-Identifier: MIT
 *
 * XCP Master.
 *
 * Implements every standard XCP command (per ASAM XCP v1.x), the level-1
 * escape sub-commands, error-code parsing, and the request/response state
 * machine on top of an XcpTransport.
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "xcp_master/xcp_protocol.h"
#include "xcp_master/xcp_transport.h"

namespace xcp_master {

// ---------------------------------------------------------------------------
// Seed-and-key compute function exported by Vector/ETAS-style XCP DLL/.so
// files.  Signature is fixed by the OEM convention:
//   uint32_t XCP_ComputeKeyFromSeed(
//       uint8_t  privilege,     // resource bit(s) to unlock
//       uint8_t* seed,          // [in]  seed bytes from GET_SEED
//       uint8_t  seed_length,
//       uint8_t* key,           // [out] computed key buffer
//       uint8_t* key_length);   // [in/out] buffer size in, key size out
// Returns 0 on success, non-zero error code on failure.
// ---------------------------------------------------------------------------
extern "C" {
using XcpComputeKeyFromSeedFn = uint32_t (*)(uint8_t privilege,
                                             uint8_t* seed,
                                             uint8_t seed_length,
                                             uint8_t* key,
                                             uint8_t* key_length);
}

constexpr const char* kSeedKeyFunctionName = "XCP_ComputeKeyFromSeed";

/// Result of a request/response transaction.
struct XcpResult {
  bool ok = false;             ///< true => positive response
  uint8_t error_code = 0;      ///< if !ok, the ERR byte returned by the slave
  std::string error_name;      ///< short name of error_code
  std::string error_description;
  std::vector<uint8_t> payload;///< response bytes following the RES/ERR PID

  static XcpResult Positive(std::vector<uint8_t> data) {
    XcpResult r;
    r.ok = true;
    r.payload = std::move(data);
    return r;
  }
  static XcpResult Negative(uint8_t err);
  static XcpResult Failure(std::string description);

  [[nodiscard]] explicit operator bool() const { return ok; }
};

struct XcpMasterConfig {
  std::chrono::milliseconds default_timeout{ 250 };
  std::chrono::milliseconds connect_timeout{ 1000 };
  std::size_t max_cto = 8;    ///< Updated from CONNECT response.
  std::size_t max_dto = 8;    ///< Updated from CONNECT response.
  ByteOrder byte_order = ByteOrder::LITTLE;
  AddressGranularity granularity = AddressGranularity::BYTE;
  bool slave_block_mode = false; ///< Slave may send multiple CTOs per UPLOAD.
  bool log_traffic = false;
};

/// Optional callback invoked for every DAQ DTO (PID < 0xFC) the master
/// observes on the transport.  Used by the application to feed measurement
/// values into A2L-decoded variables.
using DaqDtoCallback =
    std::function<void(uint8_t pid, const uint8_t* data, std::size_t size)>;

class XcpMaster {
 public:
  explicit XcpMaster(std::unique_ptr<XcpTransport> transport,
                     XcpMasterConfig cfg = {});
  ~XcpMaster();

  XcpMaster(const XcpMaster&) = delete;
  XcpMaster& operator=(const XcpMaster&) = delete;

  bool Open();
  void Close();

  const XcpMasterConfig& Config() const { return cfg_; }
  XcpMasterConfig& MutableConfig() { return cfg_; }

  void SetDaqCallback(DaqDtoCallback cb) { daq_cb_ = std::move(cb); }

  // -------------------------------------------------------------------------
  // Standard commands (master_to_slave).  Each returns an XcpResult whose
  // payload starts at the first byte after the PID (no PID byte included).
  // -------------------------------------------------------------------------
  // mode: 0 = normal, 1 = user defined.
  XcpResult Connect(uint8_t mode = 0x00);
  XcpResult Disconnect();
  XcpResult GetStatus();
  XcpResult Synch();
  XcpResult GetCommModeInfo();
  XcpResult GetId(XcpIdType id_type);
  XcpResult SetRequest(uint8_t mode, uint16_t session_config_id);
  XcpResult GetSeed(uint8_t mode, uint8_t resource);
  XcpResult Unlock(uint8_t remaining_len, const uint8_t* key, std::size_t len);
  XcpResult SetMta(uint32_t address, uint8_t addr_extension);
  XcpResult Upload(uint8_t num_elements);
  XcpResult ShortUpload(uint8_t num_elements, uint8_t addr_extension,
                        uint32_t address);
  XcpResult BuildChecksum(uint32_t block_size);
  XcpResult TransportLayerCmd(uint8_t sub_cmd,
                              const uint8_t* params = nullptr,
                              std::size_t params_len = 0);
  XcpResult UserCmd(uint8_t sub_cmd,
                    const uint8_t* params = nullptr,
                    std::size_t params_len = 0);

  // Calibration
  XcpResult Download(const uint8_t* data, std::size_t size);
  XcpResult DownloadNext(const uint8_t* data, std::size_t size);
  XcpResult DownloadMax(const uint8_t* data, std::size_t size);
  XcpResult ShortDownload(uint32_t address, uint8_t addr_extension,
                          const uint8_t* data, std::size_t size);
  XcpResult ModifyBits(uint8_t shift, uint16_t and_mask, uint16_t xor_mask);

  // Page switching
  XcpResult SetCalPage(uint8_t mode, uint8_t segment, uint8_t page);
  XcpResult GetCalPage(uint8_t access_mode, uint8_t segment);
  XcpResult GetPagProcessorInfo();
  XcpResult GetSegmentInfo(uint8_t mode, uint8_t segment, uint8_t segment_info,
                           uint8_t mapping_index);
  XcpResult GetPageInfo(uint8_t segment, uint8_t page);
  XcpResult SetSegmentMode(uint8_t mode, uint8_t segment);
  XcpResult GetSegmentMode(uint8_t segment);
  XcpResult CopyCalPage(uint8_t src_segment, uint8_t src_page,
                        uint8_t dst_segment, uint8_t dst_page);

  // DAQ
  XcpResult ClearDaqList(uint16_t daq_list);
  XcpResult SetDaqPtr(uint16_t daq_list, uint8_t odt, uint8_t odt_entry);
  XcpResult WriteDaq(uint8_t bit_offset, uint8_t size, uint8_t addr_extension,
                     uint32_t address);
  XcpResult SetDaqListMode(uint8_t mode, uint16_t daq_list,
                           uint16_t event_channel, uint8_t prescaler,
                           uint8_t priority);
  XcpResult GetDaqListMode(uint16_t daq_list);
  XcpResult StartStopDaqList(XcpStartStopMode mode, uint16_t daq_list);
  XcpResult StartStopSynch(XcpStartStopSynch mode);
  XcpResult GetDaqClock();
  XcpResult ReadDaq();
  XcpResult GetDaqProcessorInfo();
  XcpResult GetDaqResolutionInfo();
  XcpResult GetDaqListInfo(uint16_t daq_list);
  XcpResult GetDaqEventInfo(uint16_t event_channel);
  XcpResult FreeDaq();
  XcpResult AllocDaq(uint16_t daq_count);
  XcpResult AllocOdt(uint16_t daq_list, uint8_t odt_count);
  XcpResult AllocOdtEntry(uint16_t daq_list, uint8_t odt, uint8_t entry_count);

  // Flash programming
  XcpResult ProgramStart();
  XcpResult ProgramClear(uint8_t mode, uint32_t clear_range);
  XcpResult Program(const uint8_t* data, std::size_t size);
  XcpResult ProgramReset();
  XcpResult GetPgmProcessorInfo();
  XcpResult GetSectorInfo(uint8_t mode, uint8_t sector);
  XcpResult ProgramPrepare(uint16_t code_size);
  XcpResult ProgramFormat(uint8_t compression, uint8_t encryption,
                          uint8_t prog_method, uint8_t access);
  XcpResult ProgramNext(const uint8_t* data, std::size_t size);
  XcpResult ProgramMax(const uint8_t* data, std::size_t size);
  XcpResult ProgramVerify(uint8_t mode, uint16_t type, uint32_t value);

  // 1.3+ commands
  XcpResult WriteDaqMultiple(const std::vector<uint8_t>& packed_entries);
  XcpResult TimeCorrelationProperties(uint8_t set_properties,
                                      uint8_t get_properties_request,
                                      uint16_t cluster_id);
  XcpResult DtoCtrProperties(uint8_t modifier, uint16_t event_channel,
                             uint16_t related_event_channel,
                             uint8_t mode);

  // STD_CMD_LVL1 / sub-commands
  XcpResult GetVersion();
  XcpResult SetDaqPackedMode(uint16_t daq_list, uint8_t mode,
                             uint16_t timestamp_mode, uint16_t sample_count);
  XcpResult GetDaqPackedMode(uint16_t daq_list);

  // -------------------------------------------------------------------------
  // Decoded helpers for the most useful responses.
  // -------------------------------------------------------------------------
  std::optional<ConnectResponse> DecodeConnect(const XcpResult& r);
  std::optional<GetStatusResponse> DecodeGetStatus(const XcpResult& r);
  std::optional<GetCommModeInfoResponse> DecodeGetCommModeInfo(const XcpResult& r);
  std::optional<GetDaqProcessorInfoResponse> DecodeGetDaqProcessorInfo(const XcpResult& r);
  std::optional<GetDaqResolutionInfoResponse> DecodeGetDaqResolutionInfo(const XcpResult& r);

  // -------------------------------------------------------------------------
  // Block-mode helpers covering the common multi-frame DOWNLOAD / UPLOAD flow.
  // -------------------------------------------------------------------------
  /// Issue SET_MTA followed by enough DOWNLOAD / DOWNLOAD_NEXT frames to push
  /// the full buffer.  Honours master_block_mode / max_cto.
  XcpResult DownloadBlock(uint32_t address, uint8_t addr_extension,
                          const uint8_t* data, std::size_t size);

  /// Issue SET_MTA followed by UPLOAD until size bytes are read.  Uses one
  /// block-mode UPLOAD when the slave advertises SLAVE_BLOCK_MODE.
  XcpResult UploadBlock(uint32_t address, uint8_t addr_extension,
                        std::size_t size, std::vector<uint8_t>& out);

  // -------------------------------------------------------------------------
  // Seed & Key helpers.  GET_SEED and UNLOCK may both span multiple CTO
  // frames when the seed or key exceeds MAX_CTO - 2 bytes; these helpers run
  // the standard mode=0 / mode=1 (or remaining-bytes) loop per ASAM XCP v1.x.
  // -------------------------------------------------------------------------
  /// Repeatedly call GET_SEED (mode=0 then mode=1) until the full seed is
  /// collected.  On success seed_out holds the concatenated seed.  If the
  /// slave reports LENGTH = 0 on the first request, the resource is already
  /// unlocked: the call returns ok with seed_out left empty.
  XcpResult GetSeedComplete(uint8_t resource, std::vector<uint8_t>& seed_out);

  /// Send a key of arbitrary length by slicing it into MAX_CTO-2 chunks.
  /// The REMAINING byte of each UNLOCK frame is the total number of key
  /// bytes still to be transferred, *including* the bytes carried by that
  /// frame.  On success the returned XcpResult is the slave's final
  /// positive response, whose payload[0] is CURRENT_RESOURCE_PROTECTION.
  XcpResult SendKey(const uint8_t* key, std::size_t key_len);

  /// Authenticate against a single resource: run GetSeedComplete, hand the
  /// seed to `compute`, then run SendKey.  If the slave reports the resource
  /// is already unlocked (GET_SEED returns LENGTH=0), the compute callback
  /// is not invoked and the GET_SEED response is returned as-is.  On
  /// success, the returned XcpResult's payload[0] is the final
  /// CURRENT_RESOURCE_PROTECTION from UNLOCK.
  XcpResult Authenticate(uint8_t resource, XcpComputeKeyFromSeedFn compute);

  /// Authenticate by dynamically loading a shared library (.dll on Windows,
  /// .so on Linux) and resolving the standard `XCP_ComputeKeyFromSeed`
  /// symbol.  The library handle is closed before this function returns.
  /// For repeated authentications hold an external handle and use the
  /// function-pointer overload instead.
  XcpResult Authenticate(uint8_t resource, const std::string& library_path);

  // -------------------------------------------------------------------------
  // Low-level: send a raw CTO packet and wait for the response.  All higher
  // level methods funnel through this one.
  // -------------------------------------------------------------------------
  XcpResult SendCto(const std::vector<uint8_t>& packet,
                    std::chrono::milliseconds timeout = {});

  // Convert an XCP error byte to a description (also works without a Master).
  static std::string DescribeError(uint8_t code);

 private:
  XcpResult Transact(std::vector<uint8_t> packet,
                     std::chrono::milliseconds timeout);
  bool ReceiveResponse(std::vector<uint8_t>& out,
                       std::chrono::milliseconds timeout);

  std::unique_ptr<XcpTransport> transport_;
  XcpMasterConfig cfg_;
  std::mutex io_mutex_;
  DaqDtoCallback daq_cb_;
  bool opened_ = false;
};

}  // namespace xcp_master
