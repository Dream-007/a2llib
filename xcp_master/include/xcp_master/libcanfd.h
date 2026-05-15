/*
 * Copyright 2026
 * SPDX-License-Identifier: MIT
 *
 * Vendor-supplied CAN/CANFD frame structure and access functions.
 *
 * The user provides the following native types and APIs:
 *   typedef struct _TLIBCANFD { ... } TLIBCANFD, *PLIBCANFD;
 *   int tsapp_transmit_canfd_async(const PLIBCANFD ACANFD);
 *   int tsfifo_receive_canfd_msgs(const PLIBCANFD ACANFDBuffers,
 *                                 const ps32 ACANFDBufferSize,
 *                                 const s32 AIdxChn,
 *                                 const bool AIncludeTx);
 *
 * The struct definition is replicated here so the XCP Master library compiles
 * standalone. When linked into a project that already defines TLIBCANFD,
 * define XCP_MASTER_USE_EXTERNAL_CANFD before including this header and supply
 * the headers via XCP_MASTER_EXTERNAL_CANFD_HEADER.
 */

#pragma once

#include <cstdint>

#if defined(XCP_MASTER_USE_EXTERNAL_CANFD)
#  if defined(XCP_MASTER_EXTERNAL_CANFD_HEADER)
#    include XCP_MASTER_EXTERNAL_CANFD_HEADER
#  endif
#else

#ifdef __cplusplus
extern "C" {
#endif

using u8  = uint8_t;
using s32 = int32_t;
using s64 = int64_t;
using ps32 = s32*;

typedef struct _TLIBCANFD {
  u8  FIdxChn;       // channel index starting from 0
  u8  FProperties;   // [7]err [6]logged [2]ext [1]rtr [0]dir (0 RX, 1 TX)
  u8  FDLC;          // dlc 0..15
  u8  FFDProperties; // [2]ESI [1]BRS [0]EDL (1 = FD frame)
  s32 FIdentifier;   // CAN identifier
  s64 FTimeUs;       // timestamp in microseconds
  u8  FData[64];
} TLIBCANFD, *PLIBCANFD;

// Property bit helpers used by both sending and receiving sides.
constexpr u8 kCanPropDirTx       = 0x01;  // bit 0: 0 RX, 1 TX
constexpr u8 kCanPropRtr         = 0x02;  // bit 1: remote frame
constexpr u8 kCanPropExtended    = 0x04;  // bit 2: extended (29-bit) ID
constexpr u8 kCanPropErrorFrame  = 0x80;  // bit 7: error frame

constexpr u8 kCanFdPropEdl       = 0x01;  // bit 0: 1 = CAN-FD frame
constexpr u8 kCanFdPropBrs       = 0x02;  // bit 1: bit-rate switch
constexpr u8 kCanFdPropEsi       = 0x04;  // bit 2: error state indicator

#ifdef __cplusplus
}
#endif

#endif  // !XCP_MASTER_USE_EXTERNAL_CANFD

#ifdef __cplusplus

namespace xcp_master {

// Vendor transmit/receive signatures, redeclared so the XCP Master can call
// them directly when linked with the vendor driver.
extern "C" {
int tsapp_transmit_canfd_async(const PLIBCANFD ACANFD);
int tsfifo_receive_canfd_msgs(const PLIBCANFD ACANFDBuffers,
                              const ps32 ACANFDBufferSize,
                              const s32 AIdxChn,
                              const bool AIncludeTx);
}

}  // namespace xcp_master

#endif  // __cplusplus
