/*
 * Copyright 2026
 * SPDX-License-Identifier: MIT
 */

#include "xcp_master/signal_access.h"

#include <algorithm>
#include <cstring>
#include <sstream>

#include "a2l/a2lenums.h"
#include "a2l/a2lfile.h"
#include "a2l/a2lproject.h"
#include "a2l/characteristic.h"
#include "a2l/compumethod.h"
#include "a2l/measurement.h"
#include "a2l/module.h"
#include "a2l/recordlayout.h"

namespace xcp_master {

namespace {

bool MapDataType(a2l::A2lDataType t, SignalDataType& out, std::size_t& size) {
  switch (t) {
    case a2l::A2lDataType::UBYTE:        out = SignalDataType::U8;  size = 1; return true;
    case a2l::A2lDataType::SBYTE:        out = SignalDataType::S8;  size = 1; return true;
    case a2l::A2lDataType::UWORD:        out = SignalDataType::U16; size = 2; return true;
    case a2l::A2lDataType::SWORD:        out = SignalDataType::S16; size = 2; return true;
    case a2l::A2lDataType::ULONG:        out = SignalDataType::U32; size = 4; return true;
    case a2l::A2lDataType::SLONG:        out = SignalDataType::S32; size = 4; return true;
    case a2l::A2lDataType::A_UINT64:     out = SignalDataType::U64; size = 8; return true;
    case a2l::A2lDataType::A_INT64:      out = SignalDataType::S64; size = 8; return true;
    case a2l::A2lDataType::FLOAT32_IEEE: out = SignalDataType::F32; size = 4; return true;
    case a2l::A2lDataType::FLOAT64_IEEE: out = SignalDataType::F64; size = 8; return true;
    default: return false;
  }
}

bool IsBigEndian(a2l::A2lByteOrder bo) {
  // MSB_FIRST = big-endian.  MSB_LAST = little-endian.  Mid-endian variants
  // (MSB_FIRST_MSW_LAST / MSB_LAST_MSW_FIRST) are uncommon on scalars and
  // are treated as little-endian here.
  return bo == a2l::A2lByteOrder::MSB_FIRST;
}

void FillConversion(const a2l::Module& mod, const std::string& conv_name,
                    SignalDescriptor& d) {
  d.conversion_name = conv_name;
  d.conversion = SignalConversion::Identical;
  if (conv_name.empty() || conv_name == "NO_COMPU_METHOD") return;

  const a2l::CompuMethod* cm = mod.GetCompuMethod(conv_name);
  if (cm == nullptr) return;

  switch (cm->Type()) {
    case a2l::A2lConversionType::IDENTICAL:
      d.conversion = SignalConversion::Identical;
      break;
    case a2l::A2lConversionType::LINEAR: {
      // COEFFS_LINEAR a b   =>   phys = a * raw + b
      const auto& c = cm->CoeffsLinear();
      if (c.size() >= 2) {
        d.linear_a = c[0];
        d.linear_b = c[1];
        d.conversion = SignalConversion::Linear;
      } else {
        d.conversion = SignalConversion::Unsupported;
      }
      break;
    }
    case a2l::A2lConversionType::RAT_FUNC: {
      // RAT_FUNC a b c d e f   =>   raw = (a*phys^2 + b*phys + c) /
      //                                    (d*phys^2 + e*phys + f).
      // We support the linear sub-case (a = d = 0 and f != 0):
      //   raw = (b*phys + c) / f   =>   phys = (raw*f - c) / b
      // which we re-express as phys = A*raw + B with A = f/b, B = -c/b.
      const auto& c = cm->Coeffs();
      if (c.size() >= 6 && c[0] == 0.0 && c[3] == 0.0 && c[1] != 0.0) {
        d.linear_a = c[5] / c[1];
        d.linear_b = -c[2] / c[1];
        d.conversion = SignalConversion::Linear;
      } else {
        d.conversion = SignalConversion::Unsupported;
      }
      break;
    }
    default:
      d.conversion = SignalConversion::Unsupported;
      break;
  }
}

a2l::A2lByteOrder ResolveByteOrder(const a2l::A2lObject& obj,
                                   const a2l::Module& mod) {
  if (obj.ByteOrder() != a2l::A2lByteOrder::UNKNOWN) return obj.ByteOrder();
  if (mod.ModCommon().ByteOrder != a2l::A2lByteOrder::UNKNOWN) {
    return mod.ModCommon().ByteOrder;
  }
  // ASAM default when nothing is specified is MSB_LAST (little endian).
  return a2l::A2lByteOrder::MSB_LAST;
}

// Decode <size> little-endian bytes from @p in into a 64-bit container,
// applying sign extension for signed integer scalar types.
uint64_t BytesToU64(const uint8_t* in, std::size_t size, bool sign_extend) {
  uint64_t v = 0;
  for (std::size_t i = 0; i < size && i < 8; ++i) {
    v |= static_cast<uint64_t>(in[i]) << (i * 8);
  }
  if (sign_extend && size < 8 && size > 0) {
    const uint64_t sign_bit = 1ULL << (size * 8 - 1);
    if (v & sign_bit) {
      v |= ~((1ULL << (size * 8)) - 1ULL);
    }
  }
  return v;
}

void U64ToBytes(uint64_t v, uint8_t* out, std::size_t size) {
  for (std::size_t i = 0; i < size && i < 8; ++i) {
    out[i] = static_cast<uint8_t>(v >> (i * 8));
  }
}

bool IsSignedInteger(SignalDataType t) {
  return t == SignalDataType::S8 || t == SignalDataType::S16 ||
         t == SignalDataType::S32 || t == SignalDataType::S64;
}

}  // namespace

XcpSignalAccess::XcpSignalAccess(XcpMaster& master, const a2l::A2lFile& a2l)
    : master_(master) {
  IndexModule(a2l);
}

void XcpSignalAccess::IndexModule(const a2l::A2lFile& a2l) {
  for (const auto& [mod_name, module] : a2l.Project().Modules()) {
    if (!module) continue;

    for (const auto& [name, m] : module->Measurements()) {
      if (!m) continue;
      SignalDescriptor d;
      d.name = name;
      d.kind = SignalKind::Measurement;
      d.address = static_cast<uint32_t>(m->EcuAddress());
      d.address_extension = static_cast<uint8_t>(m->EcuAddressExtension());
      if (!MapDataType(m->DataType(), d.data_type, d.byte_size)) continue;
      d.big_endian = IsBigEndian(ResolveByteOrder(*m, *module));
      d.writable = m->ReadWrite();
      d.bit_mask = m->BitMask();
      FillConversion(*module, m->Conversion(), d);
      by_name_.emplace(name, std::move(d));
    }

    for (const auto& [name, c] : module->Characteristics()) {
      if (!c) continue;
      if (c->Type() != a2l::A2lCharacteristicType::VALUE) continue;

      const a2l::RecordLayout* rl = module->GetRecordLayout(c->Deposit());
      if (rl == nullptr) continue;

      SignalDescriptor d;
      d.name = name;
      d.kind = SignalKind::Characteristic;
      d.address = static_cast<uint32_t>(c->Address());
      d.address_extension = static_cast<uint8_t>(c->EcuAddressExtension());
      if (!MapDataType(rl->FncValues().DataType, d.data_type, d.byte_size)) {
        continue;
      }
      d.big_endian = IsBigEndian(ResolveByteOrder(*c, *module));
      d.writable = true;
      d.bit_mask = c->BitMask();
      FillConversion(*module, c->Conversion(), d);
      by_name_.emplace(name, std::move(d));
    }
  }
}

const SignalDescriptor* XcpSignalAccess::Find(const std::string& name) const {
  const auto it = by_name_.find(name);
  return it == by_name_.end() ? nullptr : &it->second;
}

std::vector<std::string> XcpSignalAccess::Names() const {
  std::vector<std::string> out;
  out.reserve(by_name_.size());
  for (const auto& kv : by_name_) out.push_back(kv.first);
  std::sort(out.begin(), out.end());
  return out;
}

std::size_t XcpSignalAccess::SizeOf(SignalDataType t) {
  switch (t) {
    case SignalDataType::U8:  case SignalDataType::S8:  return 1;
    case SignalDataType::U16: case SignalDataType::S16: return 2;
    case SignalDataType::U32: case SignalDataType::S32:
    case SignalDataType::F32: return 4;
    case SignalDataType::U64: case SignalDataType::S64:
    case SignalDataType::F64: return 8;
  }
  return 0;
}

double XcpSignalAccess::DecodeRawAsDouble(SignalDataType t, uint64_t raw) {
  switch (t) {
    case SignalDataType::U8:  return static_cast<double>(static_cast<uint8_t>(raw));
    case SignalDataType::S8:  return static_cast<double>(static_cast<int8_t>(raw));
    case SignalDataType::U16: return static_cast<double>(static_cast<uint16_t>(raw));
    case SignalDataType::S16: return static_cast<double>(static_cast<int16_t>(raw));
    case SignalDataType::U32: return static_cast<double>(static_cast<uint32_t>(raw));
    case SignalDataType::S32: return static_cast<double>(static_cast<int32_t>(raw));
    case SignalDataType::U64: return static_cast<double>(raw);
    case SignalDataType::S64: return static_cast<double>(static_cast<int64_t>(raw));
    case SignalDataType::F32: {
      float f;
      const uint32_t bits = static_cast<uint32_t>(raw);
      std::memcpy(&f, &bits, sizeof f);
      return static_cast<double>(f);
    }
    case SignalDataType::F64: {
      double d;
      std::memcpy(&d, &raw, sizeof d);
      return d;
    }
  }
  return 0.0;
}

uint64_t XcpSignalAccess::EncodeDoubleAsRaw(SignalDataType t, double phys) {
  switch (t) {
    case SignalDataType::U8:  return static_cast<uint64_t>(static_cast<uint8_t>(phys));
    case SignalDataType::S8:  return static_cast<uint64_t>(static_cast<int8_t>(phys)) & 0xFFULL;
    case SignalDataType::U16: return static_cast<uint64_t>(static_cast<uint16_t>(phys));
    case SignalDataType::S16: return static_cast<uint64_t>(static_cast<int16_t>(phys)) & 0xFFFFULL;
    case SignalDataType::U32: return static_cast<uint64_t>(static_cast<uint32_t>(phys));
    case SignalDataType::S32: return static_cast<uint64_t>(static_cast<int32_t>(phys)) & 0xFFFFFFFFULL;
    case SignalDataType::U64: return static_cast<uint64_t>(phys);
    case SignalDataType::S64: return static_cast<uint64_t>(static_cast<int64_t>(phys));
    case SignalDataType::F32: {
      const float f = static_cast<float>(phys);
      uint32_t bits = 0;
      std::memcpy(&bits, &f, sizeof bits);
      return bits;
    }
    case SignalDataType::F64: {
      uint64_t bits = 0;
      std::memcpy(&bits, &phys, sizeof bits);
      return bits;
    }
  }
  return 0;
}

bool XcpSignalAccess::ReadDescriptorBytes(const SignalDescriptor& d,
                                          std::vector<uint8_t>& bytes,
                                          std::string* err) {
  last_xcp_err_ = 0;
  bytes.clear();
  if (d.byte_size == 0) {
    if (err) *err = "signal '" + d.name + "': unsupported data type";
    return false;
  }
  const XcpResult r = master_.ShortUpload(
      static_cast<uint8_t>(d.byte_size), d.address_extension, d.address);
  if (!r) {
    last_xcp_err_ = r.error_code;
    if (err) {
      std::ostringstream os;
      os << "ShortUpload(" << d.name << ") failed: " << r.error_name
         << " - " << r.error_description;
      *err = os.str();
    }
    return false;
  }
  if (r.payload.size() < d.byte_size) {
    if (err) {
      std::ostringstream os;
      os << "ShortUpload(" << d.name << ") returned "
         << r.payload.size() << " bytes, expected " << d.byte_size;
      *err = os.str();
    }
    return false;
  }
  bytes.assign(r.payload.begin(), r.payload.begin() + d.byte_size);
  if (d.big_endian) std::reverse(bytes.begin(), bytes.end());
  return true;
}

bool XcpSignalAccess::ReadRaw(const std::string& name,
                              std::vector<uint8_t>& bytes,
                              std::string* err) {
  const SignalDescriptor* d = Find(name);
  if (d == nullptr) {
    if (err) *err = "unknown signal '" + name + "'";
    return false;
  }
  return ReadDescriptorBytes(*d, bytes, err);
}

bool XcpSignalAccess::ReadRawU64(const std::string& name, uint64_t& raw,
                                  std::string* err) {
  const SignalDescriptor* d = Find(name);
  if (d == nullptr) {
    if (err) *err = "unknown signal '" + name + "'";
    return false;
  }
  std::vector<uint8_t> bytes;
  if (!ReadDescriptorBytes(*d, bytes, err)) return false;
  raw = BytesToU64(bytes.data(), bytes.size(), IsSignedInteger(d->data_type));
  if (d->bit_mask != 0) {
    // Right-shift the masked bits to the lsb so callers see the field value.
    uint64_t mask = d->bit_mask;
    uint64_t shift = 0;
    while ((mask & 1ULL) == 0ULL && mask != 0ULL) {
      mask >>= 1;
      ++shift;
    }
    raw = (raw & d->bit_mask) >> shift;
  }
  return true;
}

bool XcpSignalAccess::ReadPhysical(const std::string& name, double& value,
                                    std::string* err) {
  const SignalDescriptor* d = Find(name);
  if (d == nullptr) {
    if (err) *err = "unknown signal '" + name + "'";
    return false;
  }
  uint64_t raw = 0;
  if (!ReadRawU64(name, raw, err)) return false;
  value = DecodeRawAsDouble(d->data_type, raw);
  if (d->conversion == SignalConversion::Linear) {
    value = d->linear_a * value + d->linear_b;
  }
  return true;
}

bool XcpSignalAccess::WriteDescriptorBytes(const SignalDescriptor& d,
                                            const uint8_t* data,
                                            std::size_t size,
                                            std::string* err) {
  last_xcp_err_ = 0;
  if (!d.writable) {
    if (err) *err = "signal '" + d.name + "' is not writable";
    return false;
  }
  if (size != d.byte_size) {
    if (err) {
      std::ostringstream os;
      os << "signal '" << d.name << "' expects " << d.byte_size
         << " bytes, got " << size;
      *err = os.str();
    }
    return false;
  }
  // Swap to slave byte order if needed.  We make a small copy on the stack
  // for the typical scalar case.
  uint8_t tx[8] = {0};
  std::memcpy(tx, data, size);
  if (d.big_endian) std::reverse(tx, tx + size);

  const XcpResult r =
      master_.ShortDownload(d.address, d.address_extension, tx, size);
  if (!r) {
    last_xcp_err_ = r.error_code;
    if (err) {
      std::ostringstream os;
      os << "ShortDownload(" << d.name << ") failed: " << r.error_name
         << " - " << r.error_description;
      *err = os.str();
    }
    return false;
  }
  return true;
}

bool XcpSignalAccess::WriteRaw(const std::string& name, const uint8_t* data,
                                std::size_t size, std::string* err) {
  const SignalDescriptor* d = Find(name);
  if (d == nullptr) {
    if (err) *err = "unknown signal '" + name + "'";
    return false;
  }
  if (d->bit_mask != 0) {
    if (err) *err = "signal '" + name +
                    "' is bit-masked; raw byte write not supported";
    return false;
  }
  return WriteDescriptorBytes(*d, data, size, err);
}

bool XcpSignalAccess::WriteRawU64(const std::string& name, uint64_t raw,
                                   std::string* err) {
  const SignalDescriptor* d = Find(name);
  if (d == nullptr) {
    if (err) *err = "unknown signal '" + name + "'";
    return false;
  }
  if (d->bit_mask != 0) {
    if (err) *err = "signal '" + name +
                    "' is bit-masked; not yet supported (needs RMW)";
    return false;
  }
  uint8_t buf[8] = {0};
  U64ToBytes(raw, buf, d->byte_size);
  return WriteDescriptorBytes(*d, buf, d->byte_size, err);
}

bool XcpSignalAccess::WritePhysical(const std::string& name, double value,
                                     std::string* err) {
  const SignalDescriptor* d = Find(name);
  if (d == nullptr) {
    if (err) *err = "unknown signal '" + name + "'";
    return false;
  }
  double raw_d = value;
  if (d->conversion == SignalConversion::Linear) {
    if (d->linear_a == 0.0) {
      if (err) *err = "signal '" + name +
                      "' has zero slope - cannot invert linear conversion";
      return false;
    }
    raw_d = (value - d->linear_b) / d->linear_a;
  } else if (d->conversion == SignalConversion::Unsupported) {
    if (err) *err = "signal '" + name +
                    "' uses an unsupported COMPU_METHOD; write the raw value "
                    "explicitly via WriteRawU64";
    return false;
  }
  const uint64_t raw = EncodeDoubleAsRaw(d->data_type, raw_d);
  return WriteRawU64(name, raw, err);
}

}  // namespace xcp_master
