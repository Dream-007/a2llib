/*
 * Copyright 2026
 * SPDX-License-Identifier: MIT
 *
 * XCP Protocol definitions: command codes, response codes, error codes,
 * resource flags, and bit layouts as defined by ASAM XCP v1.x.
 */

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace xcp_master {

// ---------------------------------------------------------------------------
// Standard XCP command codes (Packet Identifier byte, master -> slave).
// ---------------------------------------------------------------------------
enum class XcpCommand : uint8_t {
  // Standard
  CONNECT                     = 0xFF,
  DISCONNECT                  = 0xFE,
  GET_STATUS                  = 0xFD,
  SYNCH                       = 0xFC,
  GET_COMM_MODE_INFO          = 0xFB,
  GET_ID                      = 0xFA,
  SET_REQUEST                 = 0xF9,
  GET_SEED                    = 0xF8,
  UNLOCK                      = 0xF7,
  SET_MTA                     = 0xF6,
  UPLOAD                      = 0xF5,
  SHORT_UPLOAD                = 0xF4,
  BUILD_CHECKSUM              = 0xF3,
  TRANSPORT_LAYER_CMD         = 0xF2,
  USER_CMD                    = 0xF1,

  // Calibration
  DOWNLOAD                    = 0xF0,
  DOWNLOAD_NEXT               = 0xEF,
  DOWNLOAD_MAX                = 0xEE,
  SHORT_DOWNLOAD              = 0xED,
  MODIFY_BITS                 = 0xEC,

  // Page switching
  SET_CAL_PAGE                = 0xEB,
  GET_CAL_PAGE                = 0xEA,
  GET_PAG_PROCESSOR_INFO      = 0xE9,
  GET_SEGMENT_INFO            = 0xE8,
  GET_PAGE_INFO               = 0xE7,
  SET_SEGMENT_MODE            = 0xE6,
  GET_SEGMENT_MODE            = 0xE5,
  COPY_CAL_PAGE               = 0xE4,

  // Cyclic data exchange (DAQ)
  CLEAR_DAQ_LIST              = 0xE3,
  SET_DAQ_PTR                 = 0xE2,
  WRITE_DAQ                   = 0xE1,
  SET_DAQ_LIST_MODE           = 0xE0,
  GET_DAQ_LIST_MODE           = 0xDF,
  START_STOP_DAQ_LIST         = 0xDE,
  START_STOP_SYNCH            = 0xDD,
  GET_DAQ_CLOCK               = 0xDC,
  READ_DAQ                    = 0xDB,
  GET_DAQ_PROCESSOR_INFO      = 0xDA,
  GET_DAQ_RESOLUTION_INFO     = 0xD9,
  GET_DAQ_LIST_INFO           = 0xD8,
  GET_DAQ_EVENT_INFO          = 0xD7,
  FREE_DAQ                    = 0xD6,
  ALLOC_DAQ                   = 0xD5,
  ALLOC_ODT                   = 0xD4,
  ALLOC_ODT_ENTRY             = 0xD3,

  // Flash programming
  PROGRAM_START               = 0xD2,
  PROGRAM_CLEAR               = 0xD1,
  PROGRAM                     = 0xD0,
  PROGRAM_RESET               = 0xCF,
  GET_PGM_PROCESSOR_INFO      = 0xCE,
  GET_SECTOR_INFO             = 0xCD,
  PROGRAM_PREPARE             = 0xCC,
  PROGRAM_FORMAT              = 0xCB,
  PROGRAM_NEXT                = 0xCA,
  PROGRAM_MAX                 = 0xC9,
  PROGRAM_VERIFY              = 0xC8,

  // DAQ packed / time correlation / level 1 escape (1.3+)
  WRITE_DAQ_MULTIPLE          = 0xC7,
  TIME_CORRELATION_PROPERTIES = 0xC6,
  DTO_CTR_PROPERTIES          = 0xC5,

  // Standard escape
  STD_CMD_LVL1                = 0xC0
};

// Level 1 (sub-command) codes carried after STD_CMD_LVL1 (0xC0).
enum class XcpCmdLevel1 : uint8_t {
  GET_VERSION         = 0x00,
  SET_DAQ_PACKED_MODE = 0x01,
  GET_DAQ_PACKED_MODE = 0x02
};

// ---------------------------------------------------------------------------
// Response / event PID bytes (slave -> master).
// ---------------------------------------------------------------------------
constexpr uint8_t kPidResp = 0xFF;  // Positive response
constexpr uint8_t kPidErr  = 0xFE;  // Negative response (error)
constexpr uint8_t kPidEv   = 0xFD;  // Asynchronous event
constexpr uint8_t kPidServ = 0xFC;  // Service request packet

// ---------------------------------------------------------------------------
// Error codes (ERR byte in negative response, see XCP spec table).
// ---------------------------------------------------------------------------
enum class XcpError : uint8_t {
  ERR_CMD_SYNCH         = 0x00,
  ERR_CMD_BUSY          = 0x10,
  ERR_DAQ_ACTIVE        = 0x11,
  ERR_PGM_ACTIVE        = 0x12,
  ERR_CMD_UNKNOWN       = 0x20,
  ERR_CMD_SYNTAX        = 0x21,
  ERR_OUT_OF_RANGE      = 0x22,
  ERR_WRITE_PROTECTED   = 0x23,
  ERR_ACCESS_DENIED     = 0x24,
  ERR_ACCESS_LOCKED     = 0x25,
  ERR_PAGE_NOT_VALID    = 0x26,
  ERR_MODE_NOT_VALID    = 0x27,
  ERR_SEGMENT_NOT_VALID = 0x28,
  ERR_SEQUENCE          = 0x29,
  ERR_DAQ_CONFIG        = 0x2A,
  ERR_MEMORY_OVERFLOW   = 0x30,
  ERR_GENERIC           = 0x31,
  ERR_VERIFY            = 0x32,
  ERR_RESOURCE_TEMPORARY_NOT_ACCESSIBLE = 0x33,
  ERR_SUBCMD_UNKNOWN    = 0x34,
  ERR_TIMECORR_STATE_CHANGE = 0x35
};

/// Human-readable name for the error code byte.
std::string_view XcpErrorName(uint8_t code);

/// Human-readable description (cause + how the master should recover).
std::string_view XcpErrorDescription(uint8_t code);

// ---------------------------------------------------------------------------
// Resource bitmask (CONNECT response byte 1, GET_STATUS resource protection).
// ---------------------------------------------------------------------------
enum XcpResource : uint8_t {
  RES_CAL_PAG = 0x01,
  RES_DAQ     = 0x04,
  RES_STIM    = 0x08,
  RES_PGM     = 0x10,
  RES_DBG     = 0x20
};

// ---------------------------------------------------------------------------
// COMM_MODE_BASIC bitmask (CONNECT response byte 2).
// ---------------------------------------------------------------------------
enum XcpCommModeBasic : uint8_t {
  CMB_BYTE_ORDER          = 0x01,  // 0 = MSB last (little), 1 = MSB first (big)
  CMB_ADDRESS_GRANULARITY = 0x06,  // bits 1..2: 00 BYTE, 01 WORD, 10 DWORD
  CMB_SLAVE_BLOCK_MODE    = 0x40,
  CMB_OPTIONAL            = 0x80
};

enum class AddressGranularity : uint8_t {
  BYTE  = 0,
  WORD  = 1,
  DWORD = 2
};

// ---------------------------------------------------------------------------
// GET_ID identification type.
// ---------------------------------------------------------------------------
enum class XcpIdType : uint8_t {
  ASCII                = 0x00,
  FILENAME_ASAM_MC2    = 0x01,
  FILENAME_PATH_ASAM   = 0x02,
  URL_ASAM_MC2_FILE    = 0x03,
  ASAM_MC2_FILE        = 0x04,
  ECU_NAME             = 0x05
};

// ---------------------------------------------------------------------------
// SET_REQUEST mode bits.
// ---------------------------------------------------------------------------
enum XcpSetRequestMode : uint8_t {
  SR_STORE_CAL          = 0x01,
  SR_STORE_DAQ_NO_RESUME = 0x02,
  SR_STORE_DAQ_RESUME   = 0x04,
  SR_CLEAR_DAQ          = 0x08
};

// ---------------------------------------------------------------------------
// SET_CAL_PAGE access bits.
// ---------------------------------------------------------------------------
enum XcpCalPageMode : uint8_t {
  CP_ECU = 0x01,
  CP_XCP = 0x02,
  CP_ALL = 0x80
};

// ---------------------------------------------------------------------------
// SET_DAQ_LIST_MODE mode bits.
// ---------------------------------------------------------------------------
enum XcpDaqListMode : uint8_t {
  DLM_SELECTED  = 0x01,
  DLM_DIRECTION = 0x02,  // 0 = DAQ (slave->master), 1 = STIM (master->slave)
  DLM_TIMESTAMP = 0x10,
  DLM_PID_OFF   = 0x20
};

// ---------------------------------------------------------------------------
// START_STOP_DAQ_LIST mode.
// ---------------------------------------------------------------------------
enum class XcpStartStopMode : uint8_t {
  STOP   = 0x00,
  START  = 0x01,
  SELECT = 0x02
};

// ---------------------------------------------------------------------------
// START_STOP_SYNCH mode.
// ---------------------------------------------------------------------------
enum class XcpStartStopSynch : uint8_t {
  STOP_ALL      = 0x00,
  START_SELECT  = 0x01,
  STOP_SELECT   = 0x02,
  PREPARE_START = 0x03
};

// ---------------------------------------------------------------------------
// Generic byte order helpers.
// ---------------------------------------------------------------------------
enum class ByteOrder : uint8_t {
  LITTLE = 0,  // Intel; MSB last
  BIG    = 1   // Motorola; MSB first
};

inline void Pack16(std::vector<uint8_t>& out, uint16_t v, ByteOrder bo) {
  if (bo == ByteOrder::LITTLE) {
    out.push_back(static_cast<uint8_t>(v & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
  } else {
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>(v & 0xFF));
  }
}

inline void Pack32(std::vector<uint8_t>& out, uint32_t v, ByteOrder bo) {
  if (bo == ByteOrder::LITTLE) {
    out.push_back(static_cast<uint8_t>(v & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
  } else {
    out.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>(v & 0xFF));
  }
}

inline uint16_t Unpack16(const uint8_t* p, ByteOrder bo) {
  if (bo == ByteOrder::LITTLE) {
    return static_cast<uint16_t>(p[0]) |
           (static_cast<uint16_t>(p[1]) << 8);
  }
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) |
                               static_cast<uint16_t>(p[1]));
}

inline uint32_t Unpack32(const uint8_t* p, ByteOrder bo) {
  if (bo == ByteOrder::LITTLE) {
    return  static_cast<uint32_t>(p[0])         |
           (static_cast<uint32_t>(p[1]) << 8)  |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
  }
  return (static_cast<uint32_t>(p[0]) << 24) |
         (static_cast<uint32_t>(p[1]) << 16) |
         (static_cast<uint32_t>(p[2]) << 8)  |
          static_cast<uint32_t>(p[3]);
}

// ---------------------------------------------------------------------------
// Decoded responses for selected commands (those most useful to higher layers).
// ---------------------------------------------------------------------------
struct ConnectResponse {
  uint8_t resource = 0;
  uint8_t comm_mode_basic = 0;
  uint8_t max_cto = 0;
  uint16_t max_dto = 0;
  uint8_t protocol_layer_version = 0;
  uint8_t transport_layer_version = 0;

  [[nodiscard]] ByteOrder GetByteOrder() const {
    return (comm_mode_basic & CMB_BYTE_ORDER) ? ByteOrder::BIG : ByteOrder::LITTLE;
  }
  [[nodiscard]] AddressGranularity GetAddressGranularity() const {
    return static_cast<AddressGranularity>((comm_mode_basic & CMB_ADDRESS_GRANULARITY) >> 1);
  }
};

struct GetStatusResponse {
  uint8_t current_session_status = 0;
  uint8_t current_resource_protection = 0;
  uint8_t reserved = 0;
  uint16_t session_config_id = 0;
};

struct GetCommModeInfoResponse {
  uint8_t comm_mode_optional = 0;
  uint8_t max_bs = 0;
  uint8_t min_st = 0;
  uint8_t queue_size = 0;
  uint8_t xcp_driver_version = 0;
};

struct GetDaqProcessorInfoResponse {
  uint8_t daq_properties = 0;
  uint16_t max_daq = 0;
  uint16_t max_event_channel = 0;
  uint8_t min_daq = 0;
  uint8_t daq_key_byte = 0;
};

struct GetDaqResolutionInfoResponse {
  uint8_t granularity_odt_entry_size_daq = 0;
  uint8_t max_odt_entry_size_daq = 0;
  uint8_t granularity_odt_entry_size_stim = 0;
  uint8_t max_odt_entry_size_stim = 0;
  uint8_t timestamp_mode = 0;
  uint16_t timestamp_ticks = 0;
};

}  // namespace xcp_master
