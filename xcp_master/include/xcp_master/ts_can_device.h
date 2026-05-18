/*
 * Copyright 2026
 * SPDX-License-Identifier: MIT
 *
 * RAII wrapper around the TOSUN TSCAN (TSMaster) SDK lifecycle.
 *
 * Recommended call order per SDK docs:
 *   1. initialize_lib_tscan(true, true, false)        - one-shot, fixed args
 *   2. tscan_scan_devices(&count)                      - probe USB bus
 *   3. tscan_get_device_info(idx, &mfg, &prod, &serial)
 *   4. tscan_connect(serial, &handle)                  - obtain device handle
 *   5. tscan_config_canfd_by_baudrate(handle, channel, 500, 2000,
 *                                    lfdtISOCAN, lfdmNormal, 1)
 *
 * After step 5 the device is ready for tscan_transmit_canfd_async /
 * tsfifo_receive_canfd_msgs.  This class encapsulates the whole sequence
 * and tears it down via tscan_disconnect_by_handle / finalize_lib_tscan.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "xcp_master/libcanfd.h"

namespace xcp_master {

struct TsCanDeviceConfig {
  // Index into tscan_scan_devices' result.  Use the default (0) when only
  // one TOSUN box is attached.  When you have multiple, set explicit_serial.
  uint32_t device_index = 0;
  std::string explicit_serial;     // optional: connect by serial directly

  // Channels you intend to use (default = channel 0).  Each entry is
  // configured separately via tscan_config_canfd_by_baudrate.  Total
  // channel count on the device is 2 per the SDK docs.
  std::vector<uint8_t> channels = { 0 };

  // CAN-FD baud parameters - defaults match the user's spec.
  double arb_kbps = 500.0;
  double data_kbps = 2000.0;
  TLIBCANFDControllerType controller_type = lfdtISOCAN;
  TLIBCANFDControllerMode controller_mode = lfdmNormal;
  uint32_t enable_120ohm = 1;
};

struct TsCanDeviceInfo {
  std::string manufacturer;
  std::string product;
  std::string serial;
};

class TsCanDevice {
 public:
  TsCanDevice() = default;
  ~TsCanDevice();

  TsCanDevice(const TsCanDevice&) = delete;
  TsCanDevice& operator=(const TsCanDevice&) = delete;

  /// Run the full init/scan/connect/configure sequence in one call.
  /// Returns true on success.  On failure, last_error() carries the SDK
  /// error code (and a textual hint).
  bool Open(const TsCanDeviceConfig& cfg);

  /// Tear everything down (idempotent, also called by the destructor).
  void Close();

  /// Device handle for the SDK transmit/receive calls.  Returns 0 when not
  /// connected.
  [[nodiscard]] uint64_t Handle() const { return handle_; }

  [[nodiscard]] bool IsOpen() const { return opened_; }

  [[nodiscard]] const TsCanDeviceInfo& Info() const { return info_; }

  [[nodiscard]] const std::string& LastError() const { return last_error_; }

  /// Probe-only helper: list all attached TOSUN boxes without connecting.
  /// Returns true on success and fills `out` with one entry per device.
  /// Initialises the SDK if it hasn't been initialised yet.
  static bool Enumerate(std::vector<TsCanDeviceInfo>& out,
                        std::string* err = nullptr);

 private:
  static bool EnsureInitialized(std::string* err);
  bool Connect(const TsCanDeviceConfig& cfg);
  bool ConfigureChannels(const TsCanDeviceConfig& cfg);
  void SetError(const std::string& step, uint32_t rc);

  uint64_t handle_ = 0;
  bool opened_ = false;
  TsCanDeviceInfo info_;
  std::string last_error_;
};

}  // namespace xcp_master
