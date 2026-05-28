/*
 * Copyright 2026
 * SPDX-License-Identifier: MIT
 *
 * Example: drive an XCP-on-CAN slave through the TOSUN TSCAN SDK,
 * configured from an A2L file.
 *
 * Flow:
 *   1. Parse the A2L using liba2l.
 *   2. Pull CAN_ID_MASTER / CAN_ID_SLAVE / CAN-FD flag from the XCP IF_DATA
 *      block.  Falls back to a raw-text scan when liba2l's structured XCP
 *      scanner rejects the block (e.g. unknown LPDU_ID keyword); falls
 *      further back to XCP_CAN_ID_MASTER / SLAVE env-vars.
 *   3. Bring up the TOSUN device:
 *        initialize_lib_tscan -> tscan_scan_devices -> tscan_get_device_info
 *        -> tscan_connect      -> tscan_config_canfd_by_baudrate
 *      All wrapped inside TsCanDevice.
 *   4. Construct XcpCanTransport with that device handle.
 *   5. Open XcpMaster, run CONNECT / GET_STATUS / GET_COMM_MODE_INFO /
 *      GET_ID / GET_DAQ_PROCESSOR_INFO / GET_DAQ_RESOLUTION_INFO /
 *      DISCONNECT, decoding responses and printing the parsed error code
 *      on failure.
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
#include <utility>
#include <vector>

#include "a2l/a2lfile.h"
#include "a2l/a2lproject.h"
#include "a2l/module.h"
#include "a2l/xcp/xcpdatablock.h"
#include "a2l/xcp/xcponcan.h"

#include "xcp_master/signal_access.h"
#include "xcp_master/ts_can_device.h"
#include "xcp_master/xcp_can_transport.h"
#include "xcp_master/xcp_master.h"
#include "xcp_master/xcp_protocol.h"

namespace {

struct XcpEndpoints {
  uint32_t can_id_master = 0;
  uint32_t can_id_slave  = 0;
  bool extended_master = false;
  bool extended_slave  = false;
  bool use_canfd       = false;
  uint16_t version     = 0;
};

// -- raw text fallback ------------------------------------------------------
// liba2l's bison/flex XCP IF_DATA scanner currently lacks some keywords
// (e.g. LPDU_ID inside XCP_ON_FLX).  When the structured parse rejects a
// block we keep the raw text on the Module via AddIfData; a simple
// tokeniser is then enough to pluck CAN_ID_MASTER / CAN_ID_SLAVE / CAN_FD
// out of the XCP_ON_CAN sub-block.
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

std::optional<XcpEndpoints> ParseXcpOnCanRawText(std::string_view text) {
  const auto begin_pos = text.find("XCP_ON_CAN");
  if (begin_pos == std::string_view::npos) return std::nullopt;
  const auto end_pos = text.find("/end", begin_pos);
  if (end_pos == std::string_view::npos) return std::nullopt;
  const auto inner = text.substr(
      begin_pos + std::strlen("XCP_ON_CAN"),
      end_pos - (begin_pos + std::strlen("XCP_ON_CAN")));

  XcpEndpoints ep;
  std::size_t p = 0;
  bool first_number_seen = false;
  while (p < inner.size()) {
    std::string tok = NextToken(inner, p);
    if (tok.empty()) break;
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
        ep.can_id_master = xcp_master::CanTransportConfig::StripExtended(
            v, &ep.extended_master);
      }
    } else if (tok == "CAN_ID_SLAVE") {
      uint32_t v = 0;
      if (TryParseUInt(NextToken(inner, p), v)) {
        ep.can_id_slave = xcp_master::CanTransportConfig::StripExtended(
            v, &ep.extended_slave);
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

    const a2l::xcp::XcpDataBlock* block = module->GetXcpPlusDataBlock();
    if (block == nullptr || !block->IsOk()) {
      block = module->GetXcpDataBlock();
    }

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
    std::cerr << "Usage: " << argv[0]
              << " <input.a2l> [channel] [serial]"
              << " [--read=<signal>]... [--write=<signal>=<value>]...\n";
    return EXIT_FAILURE;
  }

  // Split positional and flag arguments so existing channel/serial usage
  // keeps working.
  std::vector<std::string> positional;
  std::vector<std::string> reads;
  reads.push_back("noiseSignal");
  reads.push_back("int16TestParam1");
  std::vector<std::pair<std::string, std::string>> writes;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a.rfind("--read=", 0) == 0) {
      reads.emplace_back(a.substr(7));
    } else if (a.rfind("--write=", 0) == 0) {
      const std::string rest = a.substr(8);
      const auto eq = rest.find('=');
      if (eq == std::string::npos) {
        std::cerr << "ignoring --write without '=': " << a << '\n';
        continue;
      }
      writes.emplace_back(rest.substr(0, eq), rest.substr(eq + 1));
    } else {
      positional.push_back(std::move(a));
    }
  }
  if (positional.empty()) {
    std::cerr << "missing <input.a2l>\n";
    return EXIT_FAILURE;
  }

  // 1. Parse A2L.
  a2l::A2lFile a2l;
  a2l.Filename(positional[0]);
  if (!a2l.ParseFile()) {
    std::cerr << "Failed to parse A2L: " << a2l.LastError() << '\n';
    return EXIT_FAILURE;
  }

  // 2. Resolve CAN endpoints.
  auto endpoints = ExtractXcpOnCan(a2l);
  if (!endpoints) {
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

  const uint8_t channel = (positional.size() > 1)
      ? static_cast<uint8_t>(std::strtoul(positional[1].c_str(), nullptr, 0))
      : 0;
  const std::string serial =
      (positional.size() > 2) ? positional[2] : std::string{};

  std::cout << "XCP-on-CAN endpoints: master=0x" << std::hex
            << endpoints->can_id_master << " slave=0x"
            << endpoints->can_id_slave << std::dec
            << (endpoints->use_canfd ? " [CAN-FD]" : " [CAN]")
            << " channel=" << static_cast<int>(channel)
            << (serial.empty() ? "" : (" serial=" + serial))
            << '\n';

  // 3. Bring up TOSUN device.
  xcp_master::TsCanDevice device;
  xcp_master::TsCanDeviceConfig dcfg;
  dcfg.explicit_serial = serial;
  dcfg.channels = { channel };
  // Use TSCAN defaults specified by the user: 500k arb / 2M data /
  // ISO CAN-FD / Normal mode / 120 Ohm enabled.
  if (!device.Open(dcfg)) {
    std::cerr << "TsCanDevice open failed: " << device.LastError() << '\n';
    return EXIT_FAILURE;
  }
  std::cout << "Connected: " << device.Info().manufacturer << " / "
            << device.Info().product << " / " << device.Info().serial
            << " (handle=0x" << std::hex << device.Handle() << std::dec
            << ")\n";

  // 4. Build the XCP transport on top of that handle.
  xcp_master::CanTransportConfig tcfg;
  tcfg.channel = channel;
  tcfg.can_id_master = endpoints->can_id_master;
  tcfg.can_id_slave = endpoints->can_id_slave;
  tcfg.extended_master = endpoints->extended_master;
  tcfg.extended_slave = endpoints->extended_slave;
  tcfg.use_canfd = endpoints->use_canfd;
  tcfg.brs = endpoints->use_canfd;

  auto transport =
      std::make_unique<xcp_master::XcpCanTransport>(device, tcfg);

  xcp_master::XcpMasterConfig mcfg;
  mcfg.default_timeout = std::chrono::milliseconds(500);
  mcfg.connect_timeout = std::chrono::milliseconds(1500);

  xcp_master::XcpMaster master(std::move(transport), mcfg);
  if (!master.Open()) {
    std::cerr << "Failed to open XCP master\n";
    return EXIT_FAILURE;
  }

  // 5. Standard discovery + status sequence.
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
              << " SLAVE_BLOCK="
              << ((decoded->comm_mode_basic & xcp_master::CMB_SLAVE_BLOCK_MODE)
                      ? "yes"
                      : "no")
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

  // -- Symbol-name driven read / write ---------------------------------------
  xcp_master::XcpSignalAccess signals(master, a2l);
  std::cout << "Indexed " << signals.Size()
            << " scalar signal(s) from the A2L\n";

  for (const auto& name : reads) {
    const xcp_master::SignalDescriptor* d = signals.Find(name);
    if (d == nullptr) {
      std::cout << "  READ " << name << ": not in A2L\n";
      continue;
    }
    uint64_t raw = 0;
    double phys = 0.0;
    std::string err;
    if (!signals.ReadRawU64(name, raw, &err)) {
      std::cout << "  READ " << name << ": FAIL - " << err << '\n';
      continue;
    }
    signals.ReadPhysical(name, phys, &err);
    std::cout << "  READ " << name << " @0x" << std::hex << d->address
              << std::dec << " raw=" << raw << " phys=" << phys << '\n';
  }

  for (const auto& [name, val_str] : writes) {
    const xcp_master::SignalDescriptor* d = signals.Find(name);
    if (d == nullptr) {
      std::cout << "  WRITE " << name << ": not in A2L\n";
      continue;
    }
    const double v = std::strtod(val_str.c_str(), nullptr);
    std::string err;
    if (!signals.WritePhysical(name, v, &err)) {
      std::cout << "  WRITE " << name << "=" << v << ": FAIL - " << err
                << '\n';
      continue;
    }
    double readback = 0.0;
    signals.ReadPhysical(name, readback, &err);
    std::cout << "  WRITE " << name << "=" << v
              << " OK (readback=" << readback << ")\n";
  }

  PrintResult("DISCONNECT", master.Disconnect());

  master.Close();
  device.Close();
  return EXIT_SUCCESS;
}
