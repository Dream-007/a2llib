/*
 * Copyright 2026
 * SPDX-License-Identifier: MIT
 */

#include "xcp_master/xcp_can_transport.h"

#include <algorithm>
#include <array>
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

XcpCanTransport::XcpCanTransport(TsCanDevice& device, CanTransportConfig cfg)
    : device_(device), cfg_(cfg) {}

XcpCanTransport::~XcpCanTransport() { Close(); }

bool XcpCanTransport::Open() {
  if (!device_.IsOpen()) return false;
  open_.store(true);
  Flush();
  return true;
}

void XcpCanTransport::Close() { open_.store(false); }

bool XcpCanTransport::BuildFrame(TLibCANFD& frame, const uint8_t* data,
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
  std::memcpy(frame.FData, data, size);
  if (length > size) {
    std::memset(frame.FData + size, cfg_.pad_byte, length - size);
  }
  frame.FDLC = LenToDlc(length);
  frame.FTimeUs = 0;
  return true;
}

bool XcpCanTransport::SendPacket(const uint8_t* data, std::size_t size) {
  if (!open_.load() || data == nullptr || size == 0) return false;
  if (device_.Handle() == 0) return false;

  TLibCANFD frame{};
  if (!BuildFrame(frame, data, size)) return false;

  std::scoped_lock lk(mutex_);
  const uint32_t rc = tscan_transmit_canfd_async(device_.Handle(), &frame);
  return rc == 0;
}

bool XcpCanTransport::ReceivePacket(std::vector<uint8_t>& out,
                                    std::chrono::milliseconds timeout) {
  if (!open_.load()) return false;
  if (device_.Handle() == 0) return false;

  const auto deadline = std::chrono::steady_clock::now() + timeout;

  std::array<TLibCANFD, kFifoBatch> buf{};
  while (std::chrono::steady_clock::now() < deadline) {
    int32_t count = static_cast<int32_t>(buf.size());
    uint32_t rc = 0;
    {
      std::scoped_lock lk(mutex_);
      if (!pending_rx_.empty()) {
        out = std::move(pending_rx_.front());
        pending_rx_.pop_front();
        return true;
      }

      rc = tsfifo_receive_canfd_msgs(device_.Handle(), buf.data(), &count,
                                     /*AChn=*/cfg_.channel,
                                     /*ARXTX=*/0);  // 0 = only RX frames
      if (rc == 0) {
        bool have_output = false;
        for (int32_t i = 0; i < count; ++i) {
          const TLibCANFD& frame = buf[i];
          if ((frame.FProperties & kCanPropDirTx) != 0) continue;
          if ((frame.FProperties & kCanPropErrorFrame) != 0) continue;

          const uint32_t id = static_cast<uint32_t>(frame.FIdentifier);
          const bool is_extended = (frame.FProperties & kCanPropExtended) != 0;
          if (is_extended != cfg_.extended_slave) continue;
          if (id != cfg_.can_id_slave) continue;

          const std::size_t len = DlcToLen(frame.FDLC);
          if (len == 0) continue;

          std::vector<uint8_t> packet(frame.FData, frame.FData + len);
          if (!have_output) {
            out = std::move(packet);
            have_output = true;
          } else {
            pending_rx_.push_back(std::move(packet));
          }
        }
        if (have_output) return true;
      }
    }
    if (rc != 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    if (count == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  return false;
}

void XcpCanTransport::Flush() {
  // Drain the FIFO by reading-and-discarding.  Pass rxtx=1 so any echo of
  // pre-existing TX frames is removed too.
  if (device_.Handle() == 0) return;
  std::array<TLibCANFD, kFifoBatch> buf{};
  {
    std::scoped_lock lk(mutex_);
    pending_rx_.clear();
  }
  for (int i = 0; i < 4; ++i) {
    int32_t count = static_cast<int32_t>(buf.size());
    std::scoped_lock lk(mutex_);
    const uint32_t rc = tsfifo_receive_canfd_msgs(device_.Handle(), buf.data(),
                                                  &count, cfg_.channel, 1);
    if (rc != 0) break;
    if (count == 0) break;
  }
}

}  // namespace xcp_master
