/*
 * Copyright 2026
 * SPDX-License-Identifier: MIT
 *
 * Example: build an XCP-on-CAN master from an A2L file.
 *
 * 1. Parse the A2L using liba2l.
 * 2. Walk every Module, fetch its XCP IF_DATA (XCP or XCP_PLUS) and extract
 *    the master / slave CAN identifiers and whether CAN-FD is used.
 * 3. Construct an XcpCanTransport with that configuration.
 * 4. Open an XcpMaster, run CONNECT, GET_STATUS, GET_COMM_MODE_INFO, GET_ID
 *    and read a few DAQ info blocks, printing the decoded responses or the
 *    parsed error code on failure.
 *
 * The actual CAN driver is supplied by the host project through
 * tsapp_transmit_canfd_async / tsfifo_receive_canfd_msgs.  When building this
 * example standalone (no driver linked), define XCP_MASTER_STUB_DRIVER to
 * pull in a no-op stub so the example links.  In that mode every command
 * will time out, which is exactly what a real master would report when the
 * ECU is offline; it confirms wiring and error-handling without any hardware.
 */

#include <chrono>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "a2l/a2lfile.h"
#include "a2l/a2lproject.h"
#include "a2l/module.h"
#include "a2l/xcp/xcpdatablock.h"
#include "a2l/xcp/xcponcan.h"

#include "xcp_master/xcp_can_transport.h"
#include "xcp_master/xcp_master.h"
#include "xcp_master/xcp_protocol.h"

#ifdef XCP_MASTER_STUB_DRIVER
extern "C" {
int tsapp_transmit_canfd_async(const PLIBCANFD /*frame*/) { return 0; }
int tsfifo_receive_canfd_msgs(const PLIBCANFD /*buffers*/, const ps32 size,
                              const s32 /*channel*/, const bool /*include_tx*/) {
  if (size) *size = 0;
  return 0;
}
}
#endif

namespace {

struct XcpEndpoints {
  uint32_t can_id_master = 0;
  uint32_t can_id_slave = 0;
  bool extended_master = false;
  bool extended_slave = false;
  bool use_canfd = false;
  uint16_t version = 0;
};

std::optional<XcpEndpoints> ExtractXcpOnCan(const a2l::A2lFile& file);

// -- raw text fallback ------------------------------------------------------
// liba2l's bison/flex XCP IF_DATA scanner currently lacks some keywords
// (e.g. LPDU_ID inside XCP_ON_FLX).  When the structured parse rejects a
// block, we keep the raw text on the Module via AddIfData, and a simple
// tokenizer is enough to pluck CAN_ID_MASTER / CAN_ID_SLAVE / CAN_FD out of
// the XCP_ON_CAN sub-block.
namespace {

// Skip whitespace and `/* ... */` comments common in A2L files.
void SkipWsAndComments(std::string_view s, std::size_t& p) {
  while (p < s.size()) {
    const char c = s[p];
    if (std::isspace(static_cast<unsigned char>(c))) {
      ++p;
    } else if (c == '/' && p + 1 < s.size() && s[p + 1] == '*') {
      p += 2;
      while (p + 1 < s.size() && !(s[p] == '*' && s[p + 1] == '/')) ++p;
      if (p + 1 < s.size()) p += 2;
    } else if (c == '/' && p + 1 < s.size() && s[p + 1] == '/') {
      while (p < s.size() && s[p] != '\n') ++p;
    } else {
      break;
    }
  }
}

std::string NextToken(std::string_view s, std::size_t& p) {
  SkipWsAndComments(s, p);
  std::string tok;
  if (p >= s.size()) return tok;
  if (s[p] == '"') {
    ++p;
    while (p < s.size() && s[p] != '"') tok += s[p++];
    if (p < s.size()) ++p;
    return tok;
  }
  while (p < s.size() &&
         !std::isspace(static_cast<unsigned char>(s[p])) && s[p] != '"') {
    tok += s[p++];
  }
  return tok;
}

bool TryParseUInt(const std::string& tok, uint32_t& out) {
  if (tok.empty()) return false;
  try {
    out = static_cast<uint32_t>(std::stoul(tok, nullptr, 0));
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace

std::optional<XcpEndpoints> ParseXcpOnCanRawText(std::string_view text) {
  // Find `/begin XCP_ON_CAN` ... `/end XCP_ON_CAN`.
  const auto begin_pos = text.find("XCP_ON_CAN");
  if (begin_pos == std::string_view::npos) return std::nullopt;
  const auto end_pos = text.find("/end", begin_pos);
  if (end_pos == std::string_view::npos) return std::nullopt;
  const auto inner = text.substr(begin_pos + std::strlen("XCP_ON_CAN"),
                                 end_pos - (begin_pos + std::strlen("XCP_ON_CAN")));

  XcpEndpoints ep;
  std::size_t p = 0;
  bool first_number_seen = false;
  while (p < inner.size()) {
    std::string tok = NextToken(inner, p);
    if (tok.empty()) break;
    // XCP_ON_CAN's first numeric token is the transport version (e.g. 0x0100).
    if (!first_number_seen) {
      uint32_t v = 0;
      if (TryParseUInt(tok, v)) {
        ep.version = static_cast<uint16_t>(v);
        first_number_seen = true;
        continue;
      }
    }
    if (tok == "CAN_ID_MASTER") {
      uint32_t v = 0;
      if (TryParseUInt(NextToken(inner, p), v)) {
        ep.can_id_master =
            xcp_master::CanTransportConfig::StripExtended(v, &ep.extended_master);
      }
    } else if (tok == "CAN_ID_SLAVE") {
      uint32_t v = 0;
      if (TryParseUInt(NextToken(inner, p), v)) {
        ep.can_id_slave =
            xcp_master::CanTransportConfig::StripExtended(v, &ep.extended_slave);
      }
    } else if (tok == "CAN_FD") {
      ep.use_canfd = true;
    }
  }
  if (ep.can_id_master == 0 && ep.can_id_slave == 0) return std::nullopt;
  return ep;
}

std::optional<XcpEndpoints> ExtractXcpOnCan(const a2l::A2lFile& file) {
  for (const auto& [module_name, module] : file.Project().Modules()) {
    if (!module) continue;

    // Prefer XCP_PLUS (1.x extension) when present, otherwise plain XCP.
    const a2l::xcp::XcpDataBlock* block = module->GetXcpPlusDataBlock();
    if (block == nullptr || !block->IsOk()) {
      block = module->GetXcpDataBlock();
    }

    // Structured parse succeeded?  Pull values from the typed XcpOnCan.
    if (block != nullptr && block->IsOk() &&
        !block->GetXcpOnCans().empty()) {
      const auto& on_can = block->GetXcpOnCans().front();
      XcpEndpoints ep;
      ep.version = block->GetVersion();
      if (on_can.GetCanIdMaster()) {
        ep.can_id_master = xcp_master::CanTransportConfig::StripExtended(
            *on_can.GetCanIdMaster(), &ep.extended_master);
      }
      if (on_can.GetCanIdSlave()) {
        ep.can_id_slave = xcp_master::CanTransportConfig::StripExtended(
            *on_can.GetCanIdSlave(), &ep.extended_slave);
      }
      ep.use_canfd = on_can.GetCanFd().has_value();
      return ep;
    }

    if (block != nullptr && !block->IsOk()) {
      std::cerr << "[warn] module " << module_name
                << " IF_DATA XCP structured parse error: "
                << block->LastError()
                << " - falling back to raw text scan\n";
    }

    // Fallback: liba2l's bison/flex XCP scanner does not yet recognise
    // every keyword (e.g. LPDU_ID inside XCP_ON_FLX); when it fails the
    // whole IF_DATA XCP block is dropped on the floor.  We still have the
    // raw block text from AddIfData, so we scan it directly for the
    // XCP_ON_CAN sub-block and pluck out CAN_ID_MASTER / CAN_ID_SLAVE.
    for (const auto& [proto, raw] : module->IfDatas()) {
      if (proto != "XCP" && proto != "XCPplus") continue;
      auto ep = ParseXcpOnCanRawText(raw);
      if (ep) return ep;
    }
  }
  return std::nullopt;
}

void PrintResult(const char* label, const xcp_master::XcpResult& r) {
  std::cout << label << ": ";
  if (r) {
    std::cout << "OK (" << r.payload.size() << " bytes)\n";
  } else {
    std::cout << "FAIL " << r.error_name << " (0x" << std::hex
              << static_cast<int>(r.error_code) << std::dec << ") - "
              << r.error_description << '\n';
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <input.a2l>\n";
    return EXIT_FAILURE;
  }

  a2l::A2lFile a2l;
  a2l.Filename(argv[1]);
  if (!a2l.ParseFile()) {
    std::cerr << "Failed to parse A2L: " << a2l.LastError() << '\n';
    return EXIT_FAILURE;
  }

  auto endpoints = ExtractXcpOnCan(a2l);
  if (!endpoints) {
    // Allow a manual override through environment variables when the A2L
    // does not contain XCP-on-CAN IF_DATA.
    const char* m = std::getenv("XCP_CAN_ID_MASTER");
    const char* s = std::getenv("XCP_CAN_ID_SLAVE");
    if (m == nullptr || s == nullptr) {
      std::cerr << "A2L has no XCP-on-CAN IF_DATA; set XCP_CAN_ID_MASTER "
                   "and XCP_CAN_ID_SLAVE to override.\n";
      return EXIT_FAILURE;
    }
    XcpEndpoints fallback;
    fallback.can_id_master = xcp_master::CanTransportConfig::StripExtended(
        static_cast<uint32_t>(std::strtoul(m, nullptr, 0)),
        &fallback.extended_master);
    fallback.can_id_slave = xcp_master::CanTransportConfig::StripExtended(
        static_cast<uint32_t>(std::strtoul(s, nullptr, 0)),
        &fallback.extended_slave);
    endpoints = fallback;
  }

  std::cout << "XCP-on-CAN endpoints: master=0x" << std::hex
            << endpoints->can_id_master << " slave=0x"
            << endpoints->can_id_slave << std::dec
            << (endpoints->use_canfd ? " [CAN-FD]" : " [CAN]")
            << " transport_version=0x" << std::hex << endpoints->version
            << std::dec << '\n';

  xcp_master::CanTransportConfig tcfg;
  tcfg.channel = 0;
  tcfg.can_id_master = endpoints->can_id_master;
  tcfg.can_id_slave = endpoints->can_id_slave;
  tcfg.extended_master = endpoints->extended_master;
  tcfg.extended_slave = endpoints->extended_slave;
  tcfg.use_canfd = endpoints->use_canfd;
  tcfg.brs = endpoints->use_canfd;

  auto transport = std::make_unique<xcp_master::XcpCanTransport>(tcfg);

  xcp_master::XcpMasterConfig mcfg;
  mcfg.default_timeout = std::chrono::milliseconds(500);
  mcfg.connect_timeout = std::chrono::milliseconds(1500);

  xcp_master::XcpMaster master(std::move(transport), mcfg);
  if (!master.Open()) {
    std::cerr << "Failed to open XCP master\n";
    return EXIT_FAILURE;
  }

  auto connect = master.Connect();
  PrintResult("CONNECT", connect);
  if (!connect) {
    master.Close();
    return EXIT_FAILURE;
  }
  if (auto decoded = master.DecodeConnect(connect)) {
    std::cout << "  MAX_CTO=" << static_cast<int>(decoded->max_cto)
              << " MAX_DTO=" << decoded->max_dto << " RESOURCE=0x"
              << std::hex << static_cast<int>(decoded->resource)
              << " COMM_BASIC=0x"
              << static_cast<int>(decoded->comm_mode_basic) << std::dec
              << " PL=" << static_cast<int>(decoded->protocol_layer_version)
              << " TL=" << static_cast<int>(decoded->transport_layer_version)
              << '\n';
  }

  auto status = master.GetStatus();
  PrintResult("GET_STATUS", status);
  if (auto decoded = master.DecodeGetStatus(status)) {
    std::cout << "  session_status=0x" << std::hex
              << static_cast<int>(decoded->current_session_status)
              << " resource_protection=0x"
              << static_cast<int>(decoded->current_resource_protection)
              << std::dec << '\n';
  }

  PrintResult("GET_COMM_MODE_INFO", master.GetCommModeInfo());
  PrintResult("GET_ID", master.GetId(xcp_master::XcpIdType::FILENAME_ASAM_MC2));
  PrintResult("GET_DAQ_PROCESSOR_INFO", master.GetDaqProcessorInfo());
  PrintResult("GET_DAQ_RESOLUTION_INFO", master.GetDaqResolutionInfo());
  PrintResult("DISCONNECT", master.Disconnect());

  master.Close();
  return EXIT_SUCCESS;
}
