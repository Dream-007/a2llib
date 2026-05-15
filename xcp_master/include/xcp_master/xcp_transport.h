/*
 * Copyright 2026
 * SPDX-License-Identifier: MIT
 *
 * XCP transport-layer abstraction.
 *
 * The Master uses this interface to send and receive XCP packets without
 * caring about the underlying physical layer (CAN, CAN-FD, Ethernet, ...).
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <span>
#include <vector>

namespace xcp_master {

class XcpTransport {
 public:
  virtual ~XcpTransport() = default;

  /// Open the transport.  Returns true on success.
  virtual bool Open() = 0;

  /// Close the transport.  Idempotent.
  virtual void Close() = 0;

  /// Send a single XCP packet (CTO master->slave).  payload contains the
  /// XCP packet starting with the PID byte (no transport header).
  /// Returns true on a successful enqueue; transport-specific framing is
  /// added by the implementation.
  virtual bool SendPacket(const uint8_t* data, std::size_t size) = 0;

  /// Wait for a single XCP packet from the slave (RES/ERR/EV/SERV or
  /// DAQ DTO).  The returned buffer is the XCP packet starting with the
  /// PID byte.
  /// Returns true if a packet was received before the timeout expired.
  virtual bool ReceivePacket(std::vector<uint8_t>& out,
                             std::chrono::milliseconds timeout) = 0;

  /// Drop any queued frames so a fresh request/response transaction starts
  /// from a clean state.
  virtual void Flush() = 0;
};

}  // namespace xcp_master
