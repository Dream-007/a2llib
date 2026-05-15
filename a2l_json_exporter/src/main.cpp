#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "a2l/a2lfile.h"
#include "a2l/a2lenums.h"
#include "a2l/a2lobject.h"
#include "a2l/a2lproject.h"
#include "a2l/a2lstructs.h"
#include "a2l/axisdescr.h"
#include "a2l/characteristic.h"
#include "a2l/compumethod.h"
#include "a2l/computab.h"
#include "a2l/compuvtab.h"
#include "a2l/compuvtabrange.h"
#include "a2l/measurement.h"
#include "a2l/module.h"

namespace {

struct ObjectState {
  bool first = true;
};

void Indent(std::ostream& out, int count) {
  for (int index = 0; index < count; ++index) {
    out.put(' ');
  }
}

std::string JsonEscape(std::string_view text) {
  std::ostringstream out;
  for (const unsigned char ch : text) {
    switch (ch) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\b':
        out << "\\b";
        break;
      case '\f':
        out << "\\f";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        if (ch < 0x20) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(ch) << std::dec << std::setfill(' ');
        } else {
          out << static_cast<char>(ch);
        }
        break;
    }
  }
  return out.str();
}

void WriteJsonString(std::ostream& out, std::string_view text) {
  out << '"' << JsonEscape(text) << '"';
}

std::string ToString(std::string_view text) {
  return std::string(text.data(), text.size());
}

std::string DoubleToString(double value) {
  std::ostringstream out;
  out << std::setprecision(17) << value;
  return out.str();
}

std::string HexString(uint64_t value) {
  std::ostringstream out;
  out << "0x" << std::uppercase << std::hex << value;
  return out.str();
}

std::string_view ByteOrderToIntelMotorola(a2l::A2lByteOrder byte_order) {
  switch (byte_order) {
    case a2l::A2lByteOrder::MSB_LAST:
    case a2l::A2lByteOrder::MSB_LAST_MSW_FIRST:
      return "intel";

    case a2l::A2lByteOrder::MSB_FIRST:
    case a2l::A2lByteOrder::MSB_FIRST_MSW_LAST:
      return "motorola";

    case a2l::A2lByteOrder::UNKNOWN:
    default:
      return "unknown";
  }
}

void FieldPrefix(std::ostream& out, ObjectState& state, int indent,
                 std::string_view key) {
  if (!state.first) {
    out << ",\n";
  }
  state.first = false;
  Indent(out, indent);
  WriteJsonString(out, key);
  out << ": ";
}

void StringField(std::ostream& out, ObjectState& state, int indent,
                 std::string_view key, std::string_view value) {
  FieldPrefix(out, state, indent, key);
  WriteJsonString(out, value);
}

void BoolField(std::ostream& out, ObjectState& state, int indent,
               std::string_view key, bool value) {
  FieldPrefix(out, state, indent, key);
  out << (value ? "true" : "false");
}

void UIntField(std::ostream& out, ObjectState& state, int indent,
               std::string_view key, uint64_t value) {
  FieldPrefix(out, state, indent, key);
  out << value;
}

void IntField(std::ostream& out, ObjectState& state, int indent,
              std::string_view key, int64_t value) {
  FieldPrefix(out, state, indent, key);
  out << value;
}

void DoubleField(std::ostream& out, ObjectState& state, int indent,
                 std::string_view key, double value) {
  FieldPrefix(out, state, indent, key);
  if (std::isfinite(value)) {
    out << DoubleToString(value);
  } else {
    out << "null";
  }
}

template <typename Fn>
void ObjectField(std::ostream& out, ObjectState& state, int indent,
                 std::string_view key, Fn write_body) {
  FieldPrefix(out, state, indent, key);
  out << "{\n";
  ObjectState child;
  write_body(child, indent + 2);
  out << '\n';
  Indent(out, indent);
  out << '}';
}

template <typename Fn>
void ArrayField(std::ostream& out, ObjectState& state, int indent,
                std::string_view key, Fn write_body) {
  FieldPrefix(out, state, indent, key);
  out << "[\n";
  write_body(indent + 2);
  out << '\n';
  Indent(out, indent);
  out << ']';
}

template <typename T>
std::vector<std::pair<std::string, const T*>> SortPtrMap(
    const std::unordered_map<std::string, std::unique_ptr<T>>& items) {
  std::vector<std::pair<std::string, const T*>> sorted;
  sorted.reserve(items.size());
  for (const auto& [name, item] : items) {
    sorted.emplace_back(name, item.get());
  }
  std::ranges::sort(sorted, {}, &std::pair<std::string, const T*>::first);
  return sorted;
}

template <typename T>
void WriteStringArray(std::ostream& out, const T& values, int indent) {
  bool first = true;
  for (const auto& value : values) {
    if (!first) {
      out << ",\n";
    }
    first = false;
    Indent(out, indent);
    WriteJsonString(out, value);
  }
}

template <typename T>
void WriteUIntArray(std::ostream& out, const T& values, int indent) {
  bool first = true;
  for (const auto value : values) {
    if (!first) {
      out << ",\n";
    }
    first = false;
    Indent(out, indent);
    out << value;
  }
}

template <typename T>
void WriteDoubleArray(std::ostream& out, const T& values, int indent) {
  bool first = true;
  for (const auto value : values) {
    if (!first) {
      out << ",\n";
    }
    first = false;
    Indent(out, indent);
    out << (std::isfinite(value) ? DoubleToString(value) : "null");
  }
}

void WriteStringMap(std::ostream& out,
                    const std::unordered_map<std::string, std::string>& values,
                    int indent) {
  std::vector<std::pair<std::string, std::string>> sorted(values.begin(),
                                                          values.end());
  std::ranges::sort(sorted, {}, &std::pair<std::string, std::string>::first);
  ObjectState state;
  for (const auto& [key, value] : sorted) {
    StringField(out, state, indent, key, value);
  }
}

void WriteAnnotations(std::ostream& out, const a2l::AnnotationList& annotations,
                      int indent) {
  bool first = true;
  for (const auto& annotation : annotations) {
    if (!first) {
      out << ",\n";
    }
    first = false;
    Indent(out, indent);
    out << "{\n";
    ObjectState state;
    StringField(out, state, indent + 2, "label", annotation.Label);
    StringField(out, state, indent + 2, "origin", annotation.Origin);
    ArrayField(out, state, indent + 2, "text",
               [&](int child_indent) {
                 WriteStringArray(out, annotation.Text, child_indent);
               });
    out << '\n';
    Indent(out, indent);
    out << '}';
  }
}

void WriteBaseObject(std::ostream& out, ObjectState& state, int indent,
                     const a2l::A2lObject& object) {
  StringField(out, state, indent, "name", object.Name());
  StringField(out, state, indent, "description", object.Description());
  StringField(out, state, indent, "display_identifier",
              object.DisplayIdentifier());
  StringField(out, state, indent, "byte_order",
              ByteOrderToIntelMotorola(object.ByteOrder()));
  StringField(out, state, indent, "calibration_access",
              ToString(a2l::CalibrationAccessToString(
                  object.CalibrationAccess())));
  BoolField(out, state, indent, "discrete", object.Discrete());
  IntField(out, state, indent, "ecu_address_extension",
           object.EcuAddressExtension());
  ObjectField(out, state, indent, "extended_limits",
              [&](ObjectState& child, int child_indent) {
                DoubleField(out, child, child_indent, "lower",
                            object.ExtendedLimits().LowerLimits);
                DoubleField(out, child, child_indent, "upper",
                            object.ExtendedLimits().UpperLimits);
              });
  StringField(out, state, indent, "format", object.Format());
  ArrayField(out, state, indent, "function_list",
             [&](int child_indent) {
               WriteStringArray(out, object.FunctionList(), child_indent);
             });
  BoolField(out, state, indent, "guard_rails", object.GuardRails());
  ArrayField(out, state, indent, "matrix_dim",
             [&](int child_indent) {
               WriteUIntArray(out, object.MatrixDim(), child_indent);
             });
  ObjectField(out, state, indent, "max_refresh",
              [&](ObjectState& child, int child_indent) {
                UIntField(out, child, child_indent, "scaling_unit",
                          object.MaxRefresh().ScalingUnit);
                UIntField(out, child, child_indent, "rate",
                          object.MaxRefresh().Rate);
              });
  StringField(out, state, indent, "model_link", object.ModelLink());
  StringField(out, state, indent, "phys_unit", object.PhysUnit());
  BoolField(out, state, indent, "read_only", object.ReadOnly());
  StringField(out, state, indent, "ref_memory_segment",
              object.RefMemorySegment());
  DoubleField(out, state, indent, "step_size", object.StepSize());
  ObjectField(out, state, indent, "symbol_link",
              [&](ObjectState& child, int child_indent) {
                StringField(out, child, child_indent, "symbol_name",
                            object.SymbolLink().SymbolName);
                IntField(out, child, child_indent, "offset",
                         object.SymbolLink().Offset);
              });
  ArrayField(out, state, indent, "annotations",
             [&](int child_indent) {
               WriteAnnotations(out, object.Annotations(), child_indent);
             });
  ObjectField(out, state, indent, "if_data",
              [&](ObjectState&, int child_indent) {
                WriteStringMap(out, object.IfDatas(), child_indent);
              });
}

void WriteMeasurement(std::ostream& out, const a2l::Measurement& measurement,
                      int indent) {
  out << "{\n";
  ObjectState state;
  StringField(out, state, indent + 2, "kind", "signal");
  StringField(out, state, indent + 2, "a2l_type", "MEASUREMENT");
  WriteBaseObject(out, state, indent + 2, measurement);
  StringField(out, state, indent + 2, "data_type",
              ToString(a2l::DataTypeToString(measurement.DataType())));
  StringField(out, state, indent + 2, "conversion", measurement.Conversion());
  UIntField(out, state, indent + 2, "resolution", measurement.Resolution());
  DoubleField(out, state, indent + 2, "accuracy", measurement.Accuracy());
  DoubleField(out, state, indent + 2, "lower_limit",
              measurement.LowerLimit());
  DoubleField(out, state, indent + 2, "upper_limit",
              measurement.UpperLimit());
  StringField(out, state, indent + 2, "address_type",
              ToString(a2l::AddressTypeToString(measurement.AddressType())));
  UIntField(out, state, indent + 2, "array_size", measurement.ArraySize());
  StringField(out, state, indent + 2, "bit_mask",
              HexString(measurement.BitMask()));
  ObjectField(out, state, indent + 2, "bit_operation",
              [&](ObjectState& child, int child_indent) {
                UIntField(out, child, child_indent, "left_shift",
                          measurement.BitOperation().LeftShift);
                UIntField(out, child, child_indent, "right_shift",
                          measurement.BitOperation().RightShift);
                BoolField(out, child, child_indent, "sign_extended",
                          measurement.BitOperation().SignExtended);
              });
  StringField(out, state, indent + 2, "ecu_address",
              HexString(measurement.EcuAddress()));
  StringField(out, state, indent + 2, "error_mask",
              HexString(measurement.ErrorMask()));
  StringField(out, state, indent + 2, "layout",
              ToString(a2l::LayoutToString(measurement.Layout())));
  BoolField(out, state, indent + 2, "read_write", measurement.ReadWrite());
  ArrayField(out, state, indent + 2, "virtuals",
             [&](int child_indent) {
               WriteStringArray(out, measurement.Virtuals(), child_indent);
             });
  out << '\n';
  Indent(out, indent);
  out << '}';
}

void WriteDependentCharacteristic(
    std::ostream& out, const a2l::A2lDependentCharacteristic& dependent,
    int indent) {
  out << "{\n";
  ObjectState state;
  StringField(out, state, indent + 2, "formula", dependent.Formula);
  ArrayField(out, state, indent + 2, "characteristics",
             [&](int child_indent) {
               WriteStringArray(out, dependent.CharacteristicList,
                                child_indent);
             });
  out << '\n';
  Indent(out, indent);
  out << '}';
}

void WriteAxisDescription(std::ostream& out, const a2l::AxisDescr& axis,
                          int indent) {
  out << "{\n";
  ObjectState state;
  WriteBaseObject(out, state, indent + 2, axis);
  StringField(out, state, indent + 2, "axis_type",
              ToString(a2l::AxisTypeToString(axis.AxisType())));
  StringField(out, state, indent + 2, "input_quantity",
              axis.InputQuantity());
  StringField(out, state, indent + 2, "conversion", axis.Conversion());
  UIntField(out, state, indent + 2, "max_axis_points",
            axis.MaxAxisPoints());
  DoubleField(out, state, indent + 2, "lower_limit", axis.LowerLimit());
  DoubleField(out, state, indent + 2, "upper_limit", axis.UpperLimit());
  StringField(out, state, indent + 2, "axis_pts_ref", axis.AxisPtsRef());
  StringField(out, state, indent + 2, "curve_axis_ref",
              axis.CurveAxisRef());
  StringField(out, state, indent + 2, "deposit",
              ToString(a2l::DepositToString(axis.Deposit())));
  ObjectField(out, state, indent + 2, "fix_axis_par",
              [&](ObjectState& child, int child_indent) {
                DoubleField(out, child, child_indent, "offset",
                            axis.FixAxisPar().Offset);
                DoubleField(out, child, child_indent, "shift",
                            axis.FixAxisPar().Shift);
                UIntField(out, child, child_indent, "no_axis_points",
                          axis.FixAxisPar().NoAxisPoints);
              });
  ObjectField(out, state, indent + 2, "fix_axis_par_dist",
              [&](ObjectState& child, int child_indent) {
                DoubleField(out, child, child_indent, "offset",
                            axis.FixAxisParDist().Offset);
                DoubleField(out, child, child_indent, "distance",
                            axis.FixAxisParDist().Distance);
                UIntField(out, child, child_indent, "no_axis_points",
                          axis.FixAxisParDist().NoAxisPoints);
              });
  ArrayField(out, state, indent + 2, "fix_axis_par_list",
             [&](int child_indent) {
               WriteDoubleArray(out, axis.FixAxisParList(), child_indent);
             });
  DoubleField(out, state, indent + 2, "max_gradient", axis.MaxGrad());
  StringField(out, state, indent + 2, "monotony",
              ToString(a2l::MonotonyToString(axis.Monotony())));
  out << '\n';
  Indent(out, indent);
  out << '}';
}

void WriteCharacteristic(std::ostream& out,
                         const a2l::Characteristic& characteristic,
                         int indent) {
  out << "{\n";
  ObjectState state;
  StringField(out, state, indent + 2, "kind", "characteristic");
  StringField(out, state, indent + 2, "a2l_type", "CHARACTERISTIC");
  WriteBaseObject(out, state, indent + 2, characteristic);
  StringField(out, state, indent + 2, "type",
              ToString(a2l::CharacteristicTypeToString(
                  characteristic.Type())));
  StringField(out, state, indent + 2, "address",
              HexString(characteristic.Address()));
  StringField(out, state, indent + 2, "bit_mask",
              HexString(characteristic.BitMask()));
  StringField(out, state, indent + 2, "comparison_quantity",
              characteristic.ComparisonQuantity());
  StringField(out, state, indent + 2, "conversion",
              characteristic.Conversion());
  FieldPrefix(out, state, indent + 2, "dependent_characteristic");
  WriteDependentCharacteristic(out, characteristic.DependentCharacteristic(),
                               indent + 2);
  StringField(out, state, indent + 2, "deposit",
              characteristic.Deposit());
  StringField(out, state, indent + 2, "encoding",
              ToString(a2l::EncodingToString(characteristic.Encoding())));
  ArrayField(out, state, indent + 2, "map_list",
             [&](int child_indent) {
               WriteStringArray(out, characteristic.MapList(), child_indent);
             });
  DoubleField(out, state, indent + 2, "max_diff",
              characteristic.MaxDiff());
  DoubleField(out, state, indent + 2, "lower_limit",
              characteristic.LowerLimit());
  DoubleField(out, state, indent + 2, "upper_limit",
              characteristic.UpperLimit());
  UIntField(out, state, indent + 2, "number", characteristic.Number());
  FieldPrefix(out, state, indent + 2, "virtual_characteristic");
  WriteDependentCharacteristic(out, characteristic.VirtualCharacteristic(),
                               indent + 2);
  ArrayField(out, state, indent + 2, "axis_descriptions",
             [&](int child_indent) {
               bool first = true;
               for (const auto& axis : characteristic.AxisDescriptions()) {
                 if (axis == nullptr) {
                   continue;
                 }
                 if (!first) {
                   out << ",\n";
                 }
                 first = false;
                 Indent(out, child_indent);
                 WriteAxisDescription(out, *axis, child_indent);
               }
             });
  out << '\n';
  Indent(out, indent);
  out << '}';
}

void WriteCompuMethod(std::ostream& out, const a2l::CompuMethod& method,
                      int indent) {
  out << "{\n";
  ObjectState state;
  WriteBaseObject(out, state, indent + 2, method);
  StringField(out, state, indent + 2, "type",
              ToString(a2l::ConversionTypeToString(method.Type())));
  ArrayField(out, state, indent + 2, "coeffs",
             [&](int child_indent) {
               WriteDoubleArray(out, method.Coeffs(), child_indent);
             });
  ArrayField(out, state, indent + 2, "coeffs_linear",
             [&](int child_indent) {
               WriteDoubleArray(out, method.CoeffsLinear(), child_indent);
             });
  StringField(out, state, indent + 2, "compu_tab_ref",
              method.CompuTabRef());
  StringField(out, state, indent + 2, "formula", method.Formula());
  StringField(out, state, indent + 2, "formula_inv", method.FormulaInv());
  StringField(out, state, indent + 2, "ref_unit", method.RefUnit());
  StringField(out, state, indent + 2, "status_string_ref",
              method.StatusStringRef());
  out << '\n';
  Indent(out, indent);
  out << '}';
}

void WriteCompuTab(std::ostream& out, const a2l::CompuTab& table, int indent) {
  out << "{\n";
  ObjectState state;
  WriteBaseObject(out, state, indent + 2, table);
  StringField(out, state, indent + 2, "type",
              ToString(a2l::ConversionTypeToString(table.Type())));
  UIntField(out, state, indent + 2, "rows", table.Rows());
  StringField(out, state, indent + 2, "default_value",
              table.DefaultValue());
  DoubleField(out, state, indent + 2, "default_value_numeric",
              table.DefaultValueNumeric());
  ObjectField(out, state, indent + 2, "values",
              [&](ObjectState& child, int child_indent) {
                std::vector<std::pair<double, double>> values(
                    table.KeyValueList().begin(), table.KeyValueList().end());
                std::ranges::sort(values, {}, &std::pair<double, double>::first);
                for (const auto& [key, value] : values) {
                  DoubleField(out, child, child_indent, DoubleToString(key),
                              value);
                }
              });
  out << '\n';
  Indent(out, indent);
  out << '}';
}

void WriteCompuVtab(std::ostream& out, const a2l::CompuVtab& table,
                    int indent) {
  out << "{\n";
  ObjectState state;
  WriteBaseObject(out, state, indent + 2, table);
  StringField(out, state, indent + 2, "type",
              ToString(a2l::ConversionTypeToString(table.Type())));
  UIntField(out, state, indent + 2, "rows", table.Rows());
  StringField(out, state, indent + 2, "default_value",
              table.DefaultValue());
  ObjectField(out, state, indent + 2, "values",
              [&](ObjectState& child, int child_indent) {
                std::vector<std::pair<double, std::string>> values(
                    table.KeyValueList().begin(), table.KeyValueList().end());
                std::ranges::sort(values, {},
                                  &std::pair<double, std::string>::first);
                for (const auto& [key, value] : values) {
                  StringField(out, child, child_indent, DoubleToString(key),
                              value);
                }
              });
  out << '\n';
  Indent(out, indent);
  out << '}';
}

void WriteCompuVtabRange(std::ostream& out,
                         const a2l::CompuVtabRange& table, int indent) {
  out << "{\n";
  ObjectState state;
  WriteBaseObject(out, state, indent + 2, table);
  UIntField(out, state, indent + 2, "rows", table.Rows());
  StringField(out, state, indent + 2, "default_value",
              table.DefaultValue());
  ObjectField(out, state, indent + 2, "ranges",
              [&](ObjectState& child, int child_indent) {
                for (const auto& [range, value] : table.KeyValueList()) {
                  const std::string key = DoubleToString(range.first) + ".." +
                                          DoubleToString(range.second);
                  StringField(out, child, child_indent, key, value);
                }
              });
  out << '\n';
  Indent(out, indent);
  out << '}';
}

template <typename T, typename Fn>
void WriteNamedPtrMap(std::ostream& out,
                      const std::unordered_map<std::string,
                                               std::unique_ptr<T>>& items,
                      int indent, Fn write_item) {
  ObjectState state;
  for (const auto& [name, item] : SortPtrMap(items)) {
    FieldPrefix(out, state, indent, name);
    if (item == nullptr) {
      out << "null";
    } else {
      write_item(*item, indent);
    }
  }
}

void WriteModule(std::ostream& out, const a2l::Module& module, int indent) {
  out << "{\n";
  ObjectState state;
  StringField(out, state, indent + 2, "name", module.Name());
  StringField(out, state, indent + 2, "description", module.Description());
  UIntField(out, state, indent + 2, "item_count",
            module.Measurements().size() + module.Characteristics().size());
  ObjectField(out, state, indent + 2, "items",
              [&](ObjectState& items, int child_indent) {
                std::set<std::string> used_keys;

                for (const auto& [name, measurement] :
                     SortPtrMap(module.Measurements())) {
                  std::string key = name;
                  if (!used_keys.insert(key).second) {
                    key += "#MEASUREMENT";
                    used_keys.insert(key);
                  }
                  FieldPrefix(out, items, child_indent, key);
                  if (measurement == nullptr) {
                    out << "null";
                  } else {
                    WriteMeasurement(out, *measurement, child_indent);
                  }
                }

                for (const auto& [name, characteristic] :
                     SortPtrMap(module.Characteristics())) {
                  std::string key = name;
                  if (!used_keys.insert(key).second) {
                    key += "#CHARACTERISTIC";
                    used_keys.insert(key);
                  }
                  FieldPrefix(out, items, child_indent, key);
                  if (characteristic == nullptr) {
                    out << "null";
                  } else {
                    WriteCharacteristic(out, *characteristic, child_indent);
                  }
                }
              });
  ObjectField(out, state, indent + 2, "conversions",
              [&](ObjectState& conv, int conv_indent) {
                ObjectField(out, conv, conv_indent, "methods",
                            [&](ObjectState&, int child_indent) {
                              WriteNamedPtrMap<a2l::CompuMethod>(
                                  out, module.CompuMethods(), child_indent,
                                  [&](const a2l::CompuMethod& method,
                                      int item_indent) {
                                    WriteCompuMethod(out, method, item_indent);
                                  });
                            });
                ObjectField(out, conv, conv_indent, "tables",
                            [&](ObjectState&, int child_indent) {
                              WriteNamedPtrMap<a2l::CompuTab>(
                                  out, module.CompuTabs(), child_indent,
                                  [&](const a2l::CompuTab& table,
                                      int item_indent) {
                                    WriteCompuTab(out, table, item_indent);
                                  });
                            });
                ObjectField(out, conv, conv_indent, "verbal_tables",
                            [&](ObjectState&, int child_indent) {
                              WriteNamedPtrMap<a2l::CompuVtab>(
                                  out, module.CompuVtabs(), child_indent,
                                  [&](const a2l::CompuVtab& table,
                                      int item_indent) {
                                    WriteCompuVtab(out, table, item_indent);
                                  });
                            });
                ObjectField(out, conv, conv_indent, "verbal_ranges",
                            [&](ObjectState&, int child_indent) {
                              WriteNamedPtrMap<a2l::CompuVtabRange>(
                                  out, module.CompuVtabRanges(), child_indent,
                                  [&](const a2l::CompuVtabRange& table,
                                      int item_indent) {
                                    WriteCompuVtabRange(out, table,
                                                        item_indent);
                                  });
                            });
              });
  out << '\n';
  Indent(out, indent);
  out << '}';
}

bool ExportJson(const std::filesystem::path& a2l_path,
                const std::filesystem::path& json_path,
                std::string& error) {
  a2l::A2lFile file;
  file.Filename(a2l_path.string());
  if (!file.ParseFile()) {
    std::ostringstream msg;
    msg << "Failed to parse A2L: " << file.LastError();
    error = msg.str();
    return false;
  }

  const auto parent = json_path.parent_path();
  if (!parent.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(parent, ec);
    if (ec) {
      error = "Failed to create output directory: " + ec.message();
      return false;
    }
  }

  std::ofstream out(json_path, std::ios::binary | std::ios::trunc);
  if (!out) {
    error = "Failed to open output file: " + json_path.string();
    return false;
  }

  out << "{\n";
  ObjectState root;
  StringField(out, root, 2, "source", a2l_path.string());
  ObjectField(out, root, 2, "a2l_version",
              [&](ObjectState& state, int indent) {
                UIntField(out, state, indent, "version",
                          file.A2lVersion().VersionNo);
                UIntField(out, state, indent, "upgrade",
                          file.A2lVersion().UpgradeNo);
              });
  ObjectField(out, root, 2, "modules",
              [&](ObjectState& state, int indent) {
                for (const auto& [name, module] : file.Project().Modules()) {
                  FieldPrefix(out, state, indent, name);
                  if (module == nullptr) {
                    out << "null";
                  } else {
                    WriteModule(out, *module, indent);
                  }
                }
              });
  out << '\n' << "}\n";

  if (!out) {
    error = "Failed while writing output file: " + json_path.string();
    return false;
  }

  return true;
}

void PrintUsage(std::string_view program) {
  std::cerr << "Usage: " << program << " <input.a2l> <output.json>\n";
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc != 3) {
    PrintUsage(argc > 0 ? argv[0] : "a2l_json_exporter");
    return EXIT_FAILURE;
  }

  const std::filesystem::path a2l_path = argv[1];
  const std::filesystem::path json_path = argv[2];

  std::string error;
  if (!ExportJson(a2l_path, json_path, error)) {
    std::cerr << error << '\n';
    return EXIT_FAILURE;
  }

  std::cout << "Exported JSON: " << json_path << '\n';
  return EXIT_SUCCESS;
}
