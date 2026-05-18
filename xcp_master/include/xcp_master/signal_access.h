/*
 * Copyright 2026
 * SPDX-License-Identifier: MIT
 *
 * Name-based read/write helper for XCP slaves.
 *
 * Builds an in-memory index of every scalar MEASUREMENT and CHARACTERISTIC
 * (type VALUE) found in an A2L file, then exposes ReadByName / WriteByName
 * APIs that resolve the symbol to (address, address_extension, data_type,
 * byte_order, conversion) and drive the XCP master with SHORT_UPLOAD /
 * SHORT_DOWNLOAD.
 *
 * Scalar-only: arrays, curves, maps and ASCII characteristics are skipped
 * during indexing.  Bit-mask measurements are exposed but the helper reads
 * the full underlying word and applies the mask; writing a bit-mask signal
 * requires a read-modify-write and is not yet implemented.
 */

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "xcp_master/xcp_master.h"

namespace a2l {
class A2lFile;
}

namespace xcp_master {

enum class SignalKind { Measurement, Characteristic };

enum class SignalDataType {
  U8, S8, U16, S16, U32, S32, U64, S64, F32, F64
};

enum class SignalConversion {
  Identical,   ///< phys = raw
  Linear,      ///< phys = a*raw + b   (a, b in linear_a / linear_b)
  Unsupported  ///< FORM / TAB_INTP / RAT_FUNC etc. - phys reads return raw
};

struct SignalDescriptor {
  std::string name;
  SignalKind kind = SignalKind::Measurement;
  uint32_t address = 0;
  uint8_t  address_extension = 0;
  SignalDataType data_type = SignalDataType::U8;
  bool big_endian = false;     ///< false => little endian (XCP default)
  bool writable = false;
  std::size_t byte_size = 0;
  uint64_t bit_mask = 0;       ///< 0 means "no mask"
  std::string conversion_name;
  SignalConversion conversion = SignalConversion::Identical;
  double linear_a = 1.0;
  double linear_b = 0.0;
};

class XcpSignalAccess {
 public:
  /// Index every scalar MEASUREMENT / CHARACTERISTIC in @p a2l.  The
  /// reference to @p master is borrowed - the caller keeps ownership.
  XcpSignalAccess(XcpMaster& master, const a2l::A2lFile& a2l);

  /// @return descriptor for @p name, or nullptr if it isn't indexed.
  const SignalDescriptor* Find(const std::string& name) const;

  /// Iterate names (mainly for debugging).
  std::vector<std::string> Names() const;
  std::size_t Size() const { return by_name_.size(); }

  // -- Reads ----------------------------------------------------------------
  /// Raw bytes as received from the slave (already host-ordered if @p name
  /// is multi-byte and big-endian on the slave - i.e. caller-visible bytes
  /// are always little-endian / host order for scalars).
  bool ReadRaw(const std::string& name, std::vector<uint8_t>& bytes,
               std::string* err = nullptr);

  /// Same as ReadRaw but the value is decoded into a 64-bit container.
  /// Sign-extension is applied for signed types.
  bool ReadRawU64(const std::string& name, uint64_t& raw,
                  std::string* err = nullptr);

  /// Read and apply the COMPU_METHOD (IDENTICAL / LINEAR).  Unsupported
  /// conversion types fall back to the raw numeric value.
  bool ReadPhysical(const std::string& name, double& value,
                    std::string* err = nullptr);

  // -- Writes ---------------------------------------------------------------
  /// Write exactly @c byte_size bytes (caller supplies host-ordered scalar).
  /// Fails for non-writable signals.
  bool WriteRaw(const std::string& name, const uint8_t* data,
                std::size_t size, std::string* err = nullptr);

  /// Same as WriteRaw but the value is provided as a 64-bit container.
  bool WriteRawU64(const std::string& name, uint64_t raw,
                   std::string* err = nullptr);

  /// Inverse of ReadPhysical.  Returns false for non-writable signals or
  /// when the conversion is not invertible (only IDENTICAL / LINEAR with
  /// non-zero slope are accepted).
  bool WritePhysical(const std::string& name, double value,
                     std::string* err = nullptr);

  // -- Static helpers -------------------------------------------------------
  static std::size_t SizeOf(SignalDataType t);
  static double DecodeRawAsDouble(SignalDataType t, uint64_t raw);
  static uint64_t EncodeDoubleAsRaw(SignalDataType t, double phys);

  /// Last protocol-level error code from the most recent transaction.
  /// 0 means no XCP error (or success).
  uint8_t LastXcpErrorCode() const { return last_xcp_err_; }

 private:
  bool ReadDescriptorBytes(const SignalDescriptor& d,
                           std::vector<uint8_t>& bytes, std::string* err);
  bool WriteDescriptorBytes(const SignalDescriptor& d, const uint8_t* data,
                            std::size_t size, std::string* err);
  void IndexModule(const a2l::A2lFile& a2l);

  XcpMaster& master_;
  std::unordered_map<std::string, SignalDescriptor> by_name_;
  uint8_t last_xcp_err_ = 0;
};

}  // namespace xcp_master
