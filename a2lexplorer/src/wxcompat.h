/*
 * Copyright 2026 Ingemar Hedvall
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <string>
#include <string_view>

#include <wx/string.h>

inline wxString WxUtf8(const char* text) {
  return wxString::FromUTF8(text == nullptr ? "" : text);
}

inline wxString WxUtf8(const std::string& text) {
  return wxString::FromUTF8(text.c_str(), text.size());
}

inline wxString WxUtf8(std::string_view text) {
  return wxString::FromUTF8(text.data(), text.size());
}
