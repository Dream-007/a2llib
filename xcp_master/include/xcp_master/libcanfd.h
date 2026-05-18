/*
 * Copyright 2026
 * SPDX-License-Identifier: MIT
 *
 * Glue between the XCP Master and the TOSUN TSMaster TSCAN SDK.
 *
 * TSCANDef.hpp ships with the SDK and defines:
 *   - the integer aliases (u8/s32/u64/...) the API uses,
 *   - the TLibCANFD frame structure,
 *   - the TLIBCANFDControllerType / TLIBCANFDControllerMode enums,
 *   - the APP_CHANNEL enum,
 *   - every `extern "C"` entry point we call (initialize_lib_tscan,
 *     tscan_scan_devices, tscan_get_device_info, tscan_connect,
 *     tscan_disconnect_by_handle, tscan_config_canfd_by_baudrate,
 *     tscan_transmit_canfd_async, tsfifo_receive_canfd_msgs, ...).
 *
 * We include it here so the transport, the device wrapper and the example
 * all see the *same* type definitions and function signatures.  The SDK
 * itself is a Linux shared library (libTSCANApiOnLinux.so) that, at runtime,
 * dlopen()s libTSH.so from the working directory - see CMakeLists.txt for
 * the staging step that copies libTSH.so next to the executable.
 */

#pragma once

#include "xcp_master/TSCANDef.hpp"

namespace xcp_master {

// Channel-property aliases (mirroring the SDK's MASK_CAN* / MASK_CANFD*
// macros) so call-sites in this project stay readable.  We don't redefine
// the bits: just expose them under the same names we used in the previous
// stub header.
constexpr u8 kCanPropDirTx      = MASK_CANProp_DIR_TX;   // 0x01
constexpr u8 kCanPropRtr        = MASK_CANProp_REMOTE;   // 0x02
constexpr u8 kCanPropExtended   = MASK_CANProp_EXTEND;   // 0x04
constexpr u8 kCanPropErrorFrame = MASK_CANProp_ERROR;    // 0x80

constexpr u8 kCanFdPropEdl      = MASK_CANFDProp_IS_FD;  // 0x01
constexpr u8 kCanFdPropBrs      = MASK_CANFDProp_IS_BRS; // 0x02
constexpr u8 kCanFdPropEsi      = MASK_CANFDProp_IS_ESI; // 0x04

}  // namespace xcp_master
