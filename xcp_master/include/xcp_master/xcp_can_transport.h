/*
 * Copyright 2026
 * SPDX-License-Identifier: MIT
 *
 * XCP on CAN / CAN-FD transport using the TOSUN TSCAN (TSMaster) SDK.
 *
 * The actual SDK entry points are
 *   uint32_t tscan_transmit_canfd_async(size_t handle, const TLibCANFD*);
 *   uint32_t tsfifo_receive_canfd_msgs (size_t handle, const TLibCANFD*,
 *                                       int32_t* count, uint8_t channel,
 *                                       uint8_t rxtx);
 * Both take an opaque device handle obtained via tscan_connect.  The
 * handle is owned by a TsCanDevice instance and passed in by reference at
 * construction time.
 *
 * Framing per ASAM XCP-on-CAN v1.x:
 *   - One XCP packet (CTO or DTO) per CAN frame, byte 0 = PID.
 *   - Master sends on CAN_ID_MASTER, listens on CAN_ID_SLAVE.
 *   - For 11-bit IDs the high bits of the 32-bit FIdentifier are 0; for
 *     29-bit IDs we set MASK_CANProp_EXTEND in FProperties.
 *   - For CAN-FD the IS_FD bit of FFDProperties is set.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

#include "xcp_master/libcanfd.h"
#include "xcp_master/ts_can_device.h"
#include "xcp_master/xcp_transport.h"

namespace xcp_master {

struct CanTransportConfig {
  uint8_t  channel        = 0;     ///< APP_CHANNEL index (0 or 1 on TOSUN box)
  uint32_t can_id_master  = 0;     ///< master->slave (CMD/STIM)
  uint32_t can_id_slave   = 0;     ///< slave->master (RES/ERR/EV/SERV/DAQ)
  bool     extended_master = false;///< 29-bit identifier for master->slave
  bool     extended_slave  = false;///< 29-bit identifier for slave->master
  bool     use_canfd      = false; ///< Use CAN-FD frame (IS_FD bit)
  bool     brs            = false; ///< Bit-rate switching for CAN-FD
  uint8_t  pad_byte       = 0x00;  ///< Padding byte when max_dlc is required
  uint8_t  max_dlc        = 8;     ///< Pad CTO frames to this DLC
  bool     pad_to_max_dlc = false; ///< Pad outgoing CTO frames

  /// Convenience: derive 29-bit flag from A2L convention (bit 31 set means
  /// extended).  Some A2L files store identifiers as 0x8000xxxx.
  static uint32_t StripExtended(uint32_t id, bool* is_extended) {
    if ((id & 0x80000000U) != 0U) {
      if (is_extended) *is_extended = true;
      return id & 0x1FFFFFFFU;
    }
    if (is_extended) *is_extended = false;
    return id & 0x7FFU;
  }
};

class XcpCanTransport final : public XcpTransport {
 public:
  /// The device must outlive the transport.  Typically a TsCanDevice is
  /// owned by main() and opened before the master is built.
  XcpCanTransport(TsCanDevice& device, CanTransportConfig cfg);
  ~XcpCanTransport() override;

  bool Open() override;
  void Close() override;

  bool SendPacket(const uint8_t* data, std::size_t size) override;
  bool ReceivePacket(std::vector<uint8_t>& out,
                     std::chrono::milliseconds timeout) override;
  void Flush() override;

  /// Update the slave CAN id after CONNECT (some A2L files declare a
  /// default slave id but the ECU returns a real one in the GET_SLAVE_ID
  /// broadcast).
  void SetCanIdSlave(uint32_t id, bool extended) {
    std::scoped_lock lk(mutex_);
    cfg_.can_id_slave = id;
    cfg_.extended_slave = extended;
  }

  [[nodiscard]] const CanTransportConfig& Config() const { return cfg_; }

  /// Convert a DLC (0..15) into the actual data length in bytes for CAN-FD.
  static uint8_t DlcToLen(uint8_t dlc);
  /// Pick a valid CAN-FD DLC (0..15) for a desired payload length.
  static uint8_t LenToDlc(std::size_t len);

 private:
  bool BuildFrame(TLibCANFD& frame, const uint8_t* data,
                  std::size_t size) const;

  TsCanDevice& device_;
  CanTransportConfig cfg_;
  std::mutex mutex_;
  std::deque<std::vector<uint8_t>> pending_rx_;
  std::atomic<bool> open_{ false };
};

}  // namespace xcp_master
