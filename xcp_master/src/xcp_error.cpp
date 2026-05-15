/*
 * Copyright 2026
 * SPDX-License-Identifier: MIT
 *
 * XCP error code translation (ASAM XCP v1.x, Part 2, table 5).
 */

#include "xcp_master/xcp_protocol.h"

namespace xcp_master {

std::string_view XcpErrorName(uint8_t code) {
  switch (code) {
    case 0x00: return "ERR_CMD_SYNCH";
    case 0x10: return "ERR_CMD_BUSY";
    case 0x11: return "ERR_DAQ_ACTIVE";
    case 0x12: return "ERR_PGM_ACTIVE";
    case 0x20: return "ERR_CMD_UNKNOWN";
    case 0x21: return "ERR_CMD_SYNTAX";
    case 0x22: return "ERR_OUT_OF_RANGE";
    case 0x23: return "ERR_WRITE_PROTECTED";
    case 0x24: return "ERR_ACCESS_DENIED";
    case 0x25: return "ERR_ACCESS_LOCKED";
    case 0x26: return "ERR_PAGE_NOT_VALID";
    case 0x27: return "ERR_MODE_NOT_VALID";
    case 0x28: return "ERR_SEGMENT_NOT_VALID";
    case 0x29: return "ERR_SEQUENCE";
    case 0x2A: return "ERR_DAQ_CONFIG";
    case 0x30: return "ERR_MEMORY_OVERFLOW";
    case 0x31: return "ERR_GENERIC";
    case 0x32: return "ERR_VERIFY";
    case 0x33: return "ERR_RESOURCE_TEMPORARY_NOT_ACCESSIBLE";
    case 0x34: return "ERR_SUBCMD_UNKNOWN";
    case 0x35: return "ERR_TIMECORR_STATE_CHANGE";
    default:   return "ERR_UNKNOWN";
  }
}

std::string_view XcpErrorDescription(uint8_t code) {
  switch (code) {
    case 0x00:
      return "Command processor synchronisation; slave reports its current "
             "state. Master should issue SYNCH and retry.";
    case 0x10:
      return "Command was not executed because the slave is busy. Retry after "
             "a short delay (>= MIN_ST).";
    case 0x11:
      return "Command rejected because DAQ is active. Stop DAQ lists before "
             "retrying.";
    case 0x12:
      return "Command rejected because PGM is active. Issue PROGRAM_RESET or "
             "complete the flashing sequence first.";
    case 0x20:
      return "Unknown command or sub-command. Check the command list returned "
             "by GET_COMM_MODE_INFO.";
    case 0x21:
      return "Command syntax invalid. Inspect packet length / parameter "
             "alignment.";
    case 0x22:
      return "A parameter is out of range. Check addresses, sizes and the "
             "value of MAX_CTO / MAX_DTO.";
    case 0x23:
      return "Write attempt to a write-protected resource. Unlock the page or "
             "change CAL/PAG access.";
    case 0x24:
      return "The currently active resource does not permit the command. "
             "Re-request resources via GET_SEED / UNLOCK.";
    case 0x25:
      return "Access to a locked resource was denied. Run the seed & key "
             "sequence before retrying.";
    case 0x26:
      return "Selected page is not valid. Activate the page with SET_CAL_PAGE "
             "first.";
    case 0x27:
      return "Selected mode is not valid for the addressed resource.";
    case 0x28:
      return "Selected segment is not valid.";
    case 0x29:
      return "Sequence error - commands were issued in the wrong order. "
             "Restart the high-level operation (e.g. CONNECT again).";
    case 0x2A:
      return "DAQ configuration is not valid; check DAQ allocation / WRITE_DAQ "
             "/ SET_DAQ_LIST_MODE.";
    case 0x30:
      return "Memory overflow. The configured DAQ list or block exceeds slave "
             "resources.";
    case 0x31:
      return "Generic error. No more specific information available.";
    case 0x32:
      return "Verification of programmed memory failed. The flashed image "
             "does not match the source.";
    case 0x33:
      return "Resource temporarily not accessible. Retry later (e.g. after a "
             "session config change).";
    case 0x34:
      return "Unknown sub-command. Check STD_CMD_LVL1 codes.";
    case 0x35:
      return "Time correlation state change rejected.";
    default:
      return "Unknown error code; consult slave documentation.";
  }
}

}  // namespace xcp_master
