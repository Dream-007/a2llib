/*
 * Copyright 2026
 * SPDX-License-Identifier: MIT
 *
 * XCP on CAN / CAN-FD transport.
 *
 * Implements XcpTransport on top of the vendor TLIBCANFD frame structure and
 * the two C functions tsapp_transmit_canfd_async / tsfifo_receive_canfd_msgs.
 *
 * Framing per ASAM XCP-on-CAN v1.x:
 *   - Each XCP packet (CTO or DTO) occupies the data field of a single CAN
 *     frame: byte 0..N-1 of the frame is the XCP packet starting with PID.
 *     There is no extra XCP header on top of a CAN frame.
 *   - The master sends on CAN_ID_MASTER and listens on CAN_ID_SLAVE.
 *   - For 11-bit identifiers, the high bit of the 32-bit ID is 0.  For 29-bit
 *     IDs, the master sets the extended flag in TLIBCANFD::FProperties bit 2.
 *     A2L can express this through bit 31 of the ID (0x80000000).
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <vector>

#include "xcp_master/libcanfd.h"
#include "xcp_master/xcp_transport.h"

namespace xcp_master {

struct CanTransportConfig {
  uint8_t  channel        = 0;     ///< TLIBCANFD::FIdxChn
  uint32_t can_id_master  = 0;     ///< master->slave (CMD/STIM)
  uint32_t can_id_slave   = 0;     ///< slave->master (RES/ERR/EV/SERV/DAQ)
  bool     extended_master = false;///< 29-bit identifier for master->slave
  bool     extended_slave  = false;///< 29-bit identifier for slave->master
  bool     use_canfd      = false; ///< CAN-FD frames (EDL = 1)
  bool     brs            = false; ///< Bit-rate switching for CAN-FD
  uint8_t  pad_byte       = 0x00;  ///< Padding byte when max_dlc is required
  uint8_t  max_dlc        = 8;     ///< Pad CTO frames to this DLC (8 or FD)
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
  explicit XcpCanTransport(CanTransportConfig cfg);
  ~XcpCanTransport() override;

  bool Open() override;
  void Close() override;

  bool SendPacket(const uint8_t* data, std::size_t size) override;
  bool ReceivePacket(std::vector<uint8_t>& out,
                     std::chrono::milliseconds timeout) override;
  void Flush() override;

  /// Update the slave CAN id after CONNECT (some A2L files declare a default
  /// slave id but the ECU returns a real one in the GET_SLAVE_ID broadcast).
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
  bool BuildFrame(TLIBCANFD& frame, const uint8_t* data, std::size_t size) const;

  CanTransportConfig cfg_;
  std::mutex mutex_;
  std::atomic<bool> open_{false};
};

}  // namespace xcp_master
