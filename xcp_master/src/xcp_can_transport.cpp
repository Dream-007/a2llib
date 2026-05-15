/*
 * Copyright 2026
 * SPDX-License-Identifier: MIT
 */

#include "xcp_master/xcp_can_transport.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>

namespace xcp_master {

namespace {

constexpr std::size_t kFifoBatch = 32;  // Frames pulled per receive call.

constexpr std::array<uint8_t, 16> kCanFdLenTable = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

}  // namespace

uint8_t XcpCanTransport::DlcToLen(uint8_t dlc) {
  return dlc < kCanFdLenTable.size() ? kCanFdLenTable[dlc] : 0;
}

uint8_t XcpCanTransport::LenToDlc(std::size_t len) {
  for (uint8_t i = 0; i < kCanFdLenTable.size(); ++i) {
    if (kCanFdLenTable[i] >= len) return i;
  }
  return 15;  // 64 bytes
}

XcpCanTransport::XcpCanTransport(CanTransportConfig cfg) : cfg_(cfg) {}

XcpCanTransport::~XcpCanTransport() { Close(); }

bool XcpCanTransport::Open() {
  open_.store(true);
  Flush();
  return true;
}

void XcpCanTransport::Close() { open_.store(false); }

bool XcpCanTransport::BuildFrame(TLIBCANFD& frame, const uint8_t* data,
                                 std::size_t size) const {
  if (data == nullptr || size == 0) return false;

  const std::size_t cap = cfg_.use_canfd ? 64u : 8u;
  if (size > cap) return false;

  std::memset(&frame, 0, sizeof(frame));
  frame.FIdxChn = cfg_.channel;
  frame.FIdentifier = static_cast<int32_t>(cfg_.can_id_master);

  uint8_t props = kCanPropDirTx;  // master always transmits.
  if (cfg_.extended_master) props |= kCanPropExtended;
  frame.FProperties = props;

  uint8_t fd_props = 0;
  if (cfg_.use_canfd) {
    fd_props |= kCanFdPropEdl;
    if (cfg_.brs) fd_props |= kCanFdPropBrs;
  }
  frame.FFDProperties = fd_props;

  std::size_t length = size;
  if (cfg_.pad_to_max_dlc) {
    const uint8_t target_len = DlcToLen(cfg_.max_dlc);
    if (target_len > length && target_len <= cap) {
      length = target_len;
    }
  }
  // Pad the unused bytes deterministically; some ECUs require it.
  std::memcpy(frame.FData, data, size);
  if (length > size) {
    std::memset(frame.FData + size, cfg_.pad_byte, length - size);
  }

  frame.FDLC = LenToDlc(length);
  return true;
}

bool XcpCanTransport::SendPacket(const uint8_t* data, std::size_t size) {
  if (!open_.load() || data == nullptr || size == 0) return false;

  TLIBCANFD frame{};
  if (!BuildFrame(frame, data, size)) return false;

  std::scoped_lock lk(mutex_);
  const int rc = tsapp_transmit_canfd_async(&frame);
  return rc == 0;
}

bool XcpCanTransport::ReceivePacket(std::vector<uint8_t>& out,
                                    std::chrono::milliseconds timeout) {
  if (!open_.load()) return false;

  const auto deadline = std::chrono::steady_clock::now() + timeout;

  std::array<TLIBCANFD, kFifoBatch> buf{};
  while (std::chrono::steady_clock::now() < deadline) {
    int32_t count = static_cast<int32_t>(buf.size());
    int rc = 0;
    {
      std::scoped_lock lk(mutex_);
      rc = tsfifo_receive_canfd_msgs(buf.data(), &count, cfg_.channel,
                                     /*AIncludeTx=*/false);
    }
    if (rc != 0) {
      // Driver reports an error - retry briefly until the deadline.
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    for (int32_t i = 0; i < count; ++i) {
      const TLIBCANFD& frame = buf[i];
      if ((frame.FProperties & kCanPropDirTx) != 0) continue;  // ignore TX echoes
      if ((frame.FProperties & kCanPropErrorFrame) != 0) continue;

      const uint32_t id = static_cast<uint32_t>(frame.FIdentifier);
      const bool is_extended = (frame.FProperties & kCanPropExtended) != 0;
      if (is_extended != cfg_.extended_slave) continue;
      if (id != cfg_.can_id_slave) continue;

      const std::size_t len = DlcToLen(frame.FDLC);
      if (len == 0) continue;
      out.assign(frame.FData, frame.FData + len);
      return true;
    }

    if (count == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  return false;
}

void XcpCanTransport::Flush() {
  // Drain the FIFO by reading-and-discarding.
  std::array<TLIBCANFD, kFifoBatch> buf{};
  for (int i = 0; i < 4; ++i) {
    int32_t count = static_cast<int32_t>(buf.size());
    std::scoped_lock lk(mutex_);
    if (tsfifo_receive_canfd_msgs(buf.data(), &count, cfg_.channel, true) != 0) {
      break;
    }
    if (count == 0) break;
  }
}

}  // namespace xcp_master
