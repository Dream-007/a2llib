/*
 * Copyright 2026
 * SPDX-License-Identifier: MIT
 */

#include "xcp_master/ts_can_device.h"

#include <atomic>
#include <mutex>
#include <sstream>

namespace xcp_master {

namespace {

std::mutex g_init_mutex;
bool g_initialized = false;

std::string CopyCStr(char* s) {
  if (s == nullptr) return {};
  return std::string(s);
}

}  // namespace

bool TsCanDevice::EnsureInitialized(std::string* err) {
  std::scoped_lock lk(g_init_mutex);
  if (g_initialized) return true;
  // Fixed argument set per the SDK guidance: FIFO ON, error frames ON,
  // "Turbo" OFF.
  initialize_lib_tscan(true, true, false);
  g_initialized = true;
  (void)err;
  return true;
}

bool TsCanDevice::Enumerate(std::vector<TsCanDeviceInfo>& out,
                            std::string* err) {
  out.clear();
  if (!EnsureInitialized(err)) return false;

  uint32_t count = 0;
  const uint32_t rc = tscan_scan_devices(&count);
  if (rc != 0) {
    if (err) {
      std::ostringstream os;
      os << "tscan_scan_devices rc=" << rc;
      *err = os.str();
    }
    return false;
  }

  for (uint32_t i = 0; i < count; ++i) {
    char* mfg = nullptr;
    char* prod = nullptr;
    char* serial = nullptr;
    const uint32_t info_rc = tscan_get_device_info(i, &mfg, &prod, &serial);
    if (info_rc != 0) {
      if (err) {
        std::ostringstream os;
        os << "tscan_get_device_info(" << i << ") rc=" << info_rc;
        *err = os.str();
      }
      return false;
    }
    out.push_back({ CopyCStr(mfg), CopyCStr(prod), CopyCStr(serial) });
  }
  return true;
}

void TsCanDevice::SetError(const std::string& step, uint32_t rc) {
  std::ostringstream os;
  os << step << " rc=" << rc;
  if (rc == 100) {
    os << " (error 100 usually means libTSH.so was not found - confirm it is "
          "next to the executable / working directory)";
  }
  last_error_ = os.str();
}

bool TsCanDevice::Connect(const TsCanDeviceConfig& cfg) {
  std::vector<TsCanDeviceInfo> devices;
  std::string enum_err;
  if (!Enumerate(devices, &enum_err)) {
    last_error_ = "enumerate failed: " + enum_err;
    return false;
  }

  std::string serial = cfg.explicit_serial;
  if (serial.empty()) {
    if (devices.empty()) {
      last_error_ = "no TOSUN CAN devices found";
      return false;
    }
    if (cfg.device_index >= devices.size()) {
      std::ostringstream os;
      os << "device_index " << cfg.device_index << " out of range ("
         << devices.size() << " device(s) attached)";
      last_error_ = os.str();
      return false;
    }
    serial = devices[cfg.device_index].serial;
    info_ = devices[cfg.device_index];
  } else {
    for (const auto& d : devices) {
      if (d.serial == serial) {
        info_ = d;
        break;
      }
    }
  }

  uint64_t handle = 0;
  const uint32_t rc =
      tscan_connect(serial.empty() ? nullptr : serial.c_str(), &handle);
  if (rc != 0 || handle == 0) {
    SetError("tscan_connect(" + serial + ")", rc);
    return false;
  }
  handle_ = handle;
  return true;
}

bool TsCanDevice::ConfigureChannels(const TsCanDeviceConfig& cfg) {
  for (uint8_t ch : cfg.channels) {
    const uint32_t rc = tscan_config_canfd_by_baudrate(
        handle_, static_cast<APP_CHANNEL>(ch), cfg.arb_kbps, cfg.data_kbps,
        cfg.controller_type, cfg.controller_mode, cfg.enable_120ohm);
    if (rc != 0) {
      std::ostringstream os;
      os << "tscan_config_canfd_by_baudrate(channel=" << static_cast<int>(ch)
         << ", arb=" << cfg.arb_kbps << "k, data=" << cfg.data_kbps << "k)";
      SetError(os.str(), rc);
      return false;
    }
  }
  return true;
}

bool TsCanDevice::Open(const TsCanDeviceConfig& cfg) {
  if (opened_) return true;
  last_error_.clear();

  if (!EnsureInitialized(&last_error_)) return false;
  if (!Connect(cfg)) {
    Close();
    return false;
  }
  if (!ConfigureChannels(cfg)) {
    Close();
    return false;
  }
  opened_ = true;
  return true;
}

void TsCanDevice::Close() {
  if (handle_ != 0) {
    tscan_disconnect_by_handle(handle_);
    handle_ = 0;
  }
  opened_ = false;
  // We deliberately don't call finalize_lib_tscan here - other devices on
  // the same process may still be alive.  The SDK's finalize is best
  // invoked at process exit, which the user code can do explicitly.
}

TsCanDevice::~TsCanDevice() { Close(); }

}  // namespace xcp_master
