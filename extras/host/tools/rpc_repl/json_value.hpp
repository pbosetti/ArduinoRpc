// SPDX-License-Identifier: Apache-2.0
/// \file json_value.hpp
/// \brief `serial_rpc::Value <-> nlohmann::json` conversion, plus the REPL's
///   command-line tokenizer (docs/PLAN.md component 7): `parse_args()` turns
///   a typed line like `cfg {"kp": 1.5, "ki": [0, 1]}` into a method name and
///   a `std::vector<serial_rpc::Value>` of arguments.
///
/// We use `nlohmann::ordered_json` (not the default `nlohmann::json`)
/// everywhere in this header: the default alias keeps object members in a
/// `std::map<std::string, json>`, which iterates in *alphabetical* key
/// order, not insertion order. `serial_rpc::Value::Map` is an *ordered*
/// `vector<pair<Value,Value>>` (docs/PLAN.md component 5), so using
/// `ordered_json` keeps a JSON object's key order intact on every
/// Value<->JSON round trip.
///
/// ## The `bin` <-> JSON convention
/// MessagePack's `bin` (a byte string) has no native JSON equivalent. We
/// represent it as a single-key JSON object `{"$bin": "<lowercase hex>"}`
/// rather than a plain JSON array of byte values: an array of small
/// integers is indistinguishable from an actual msgpack `array` of ints, so
/// round-tripping through JSON would be ambiguous (or would silently turn a
/// `bin` into an `array` and back). The tagged-object form is unambiguous
/// and round-trips exactly. `json_to_value()` recognizes this shape (an
/// object with exactly one key, `"$bin"`, holding a valid even-length hex
/// string) and decodes it back to `Value::Bin`; any other object is decoded
/// as a `Value` map.
///
/// ## Map keys
/// `Value::Map` entries may have non-string keys (or duplicate keys) --
/// MessagePack allows it, even though it's unusual on the wire. JSON object
/// keys must be strings, so `value_to_json()` stringifies non-string keys
/// with `Value::to_string()` (e.g. the integer key `1` becomes the object
/// key `"1"`). This is lossy for pathological maps (e.g. a map with both a
/// string key `"1"` and an integer key `1` collides), which is an accepted
/// limitation of representing an arbitrary msgpack map as JSON.
#pragma once

#include "serial_rpc.hpp"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace rpc_repl {

/// See this file's top comment: object key order matters for round trips.
using json = nlohmann::ordered_json;

/// Thrown by `tokenize()`/`parse_args()` on a malformed command line: an
/// unterminated quote, or `[...]`/`{...}` brackets that never balance.
class ParseError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

// ===========================================================================
// Value <-> JSON
// ===========================================================================

namespace detail {

inline std::string bin_to_hex(const serial_rpc::Value::Bin &bin) {
  static constexpr char kDigits[] = "0123456789abcdef";
  std::string out;
  out.reserve(bin.size() * 2);
  for (uint8_t b : bin) {
    out.push_back(kDigits[b >> 4]);
    out.push_back(kDigits[b & 0x0F]);
  }
  return out;
}

/// Returns `std::nullopt` if `hex` isn't an even-length string of hex
/// digits, rather than throwing: this is used to *probe* whether a
/// `{"$bin": "..."}` object is well-formed, so a malformed one can fall back
/// to being decoded as an ordinary map instead of raising an error.
inline std::optional<serial_rpc::Value::Bin> hex_to_bin(std::string_view hex) {
  if (hex.size() % 2 != 0) return std::nullopt;
  auto nibble = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  };
  serial_rpc::Value::Bin out;
  out.reserve(hex.size() / 2);
  for (size_t i = 0; i < hex.size(); i += 2) {
    const int hi = nibble(hex[i]);
    const int lo = nibble(hex[i + 1]);
    if (hi < 0 || lo < 0) return std::nullopt;
    out.push_back(static_cast<uint8_t>((hi << 4) | lo));
  }
  return out;
}

} // namespace detail

/// Converts a `serial_rpc::Value` to its `nlohmann::ordered_json`
/// equivalent. See this file's top comment for the `bin` and map-key
/// conventions.
inline json value_to_json(const serial_rpc::Value &v) {
  using serial_rpc::Value;

  if (v.is_nil()) return json(nullptr);
  if (v.is_bool()) return json(v.as<bool>());
  if (v.is_int()) return json(v.as<int64_t>());
  if (v.is_uint()) return json(v.as<uint64_t>());
  if (v.is_double()) return json(v.as<double>());
  if (v.is_string()) return json(v.as<std::string>());
  if (v.is_bin()) {
    json obj = json::object();
    obj["$bin"] = detail::bin_to_hex(v.as<Value::Bin>());
    return obj;
  }
  if (v.is_array()) {
    json arr = json::array();
    for (const Value &e : v.as<std::vector<Value>>()) arr.push_back(value_to_json(e));
    return arr;
  }
  if (v.is_map()) {
    json obj = json::object();
    for (const auto &kv : v.map()) {
      const std::string key = kv.first.is_string() ? kv.first.as<std::string>() : kv.first.to_string();
      obj[key] = value_to_json(kv.second);
    }
    return obj;
  }
  return json(nullptr); // unreachable: every Value alternative is handled above.
}

/// Converts a `nlohmann::ordered_json` to its `serial_rpc::Value`
/// equivalent, recognizing the `{"$bin": "<hex>"}` convention documented at
/// the top of this file.
inline serial_rpc::Value json_to_value(const json &j) {
  using serial_rpc::Value;

  switch (j.type()) {
    case json::value_t::null:
      return Value();
    case json::value_t::boolean:
      return Value(j.get<bool>());
    case json::value_t::number_integer:
      return Value(j.get<int64_t>());
    case json::value_t::number_unsigned:
      return Value(j.get<uint64_t>());
    case json::value_t::number_float:
      return Value(j.get<double>());
    case json::value_t::string:
      return Value(j.get<std::string>());
    case json::value_t::array: {
      Value::Array arr;
      arr.reserve(j.size());
      for (const auto &e : j) arr.push_back(json_to_value(e));
      return Value(std::move(arr));
    }
    case json::value_t::object: {
      if (j.size() == 1) {
        const auto it = j.find("$bin");
        if (it != j.end() && it->is_string()) {
          if (auto bin = detail::hex_to_bin(it->get<std::string>())) return Value(std::move(*bin));
        }
      }
      Value::Map m;
      m.reserve(j.size());
      for (auto it = j.begin(); it != j.end(); ++it) m.emplace_back(Value(it.key()), json_to_value(it.value()));
      return Value(std::move(m));
    }
    case json::value_t::binary:
    case json::value_t::discarded:
    default:
      // Never produced by json::parse() on text input; nil is the safest
      // fallback rather than throwing on something that can't occur here.
      return Value();
  }
}

// ===========================================================================
// Command-line tokenizer + parse_args()
// ===========================================================================

namespace detail {

/// One token from `tokenize()`. `forced_string` is true for a token that
/// was shell-quoted (`"..."` or `'...'`): per docs/PLAN.md, a quoted token
/// is always taken as a literal string and never attempted as JSON, even if
/// its contents happen to look like JSON.
struct Token {
  std::string text;
  bool forced_string = false;
};

inline bool is_space(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

/// Shell-like tokenizer (docs/PLAN.md component 7): splits `line` on
/// whitespace, except that:
///  - A token starting with `"` or `'` runs until the matching unescaped
///    quote of the same kind; `\` escapes the next character (dropping the
///    backslash), so `\"` inside a double-quoted token yields a literal
///    `"`. The quotes themselves are not part of the resulting text, and
///    such a token is always `forced_string`.
///  - Otherwise, `[` / `{` open a bracket span and `]` / `}` close one;
///    while any bracket is open, whitespace does not end the token, so
///    `{"kp": 1.5, "ki": [0, 1]}` is one token. A `"` encountered while a
///    bracket is open starts a nested JSON-string span (with its own `\`
///    escaping) during which brackets and whitespace are inert, so a
///    space inside a JSON string doesn't split the token and a `}` inside
///    one doesn't close the bracket span early. `\` outside of a bracket
///    span escapes the next character (dropping the backslash), so
///    `foo\ bar` is one token `foo bar` and `foo\[bar` is `foo[bar` with no
///    bracket ever opened.
/// Throws ParseError if a quote or a bracket span is never closed, or if a
/// closing bracket appears with none open.
inline std::vector<Token> tokenize(std::string_view line) {
  std::vector<Token> tokens;
  size_t i = 0;
  const size_t n = line.size();

  while (i < n) {
    while (i < n && is_space(line[i])) ++i;
    if (i >= n) break;

    Token tok;
    if (line[i] == '"' || line[i] == '\'') {
      const char quote = line[i];
      ++i;
      tok.forced_string = true;
      bool closed = false;
      while (i < n) {
        const char c = line[i];
        if (c == '\\' && i + 1 < n) {
          tok.text.push_back(line[i + 1]);
          i += 2;
          continue;
        }
        if (c == quote) {
          ++i;
          closed = true;
          break;
        }
        tok.text.push_back(c);
        ++i;
      }
      if (!closed) throw ParseError("rpc_repl: unterminated quote in command line");
    } else {
      int depth = 0;
      bool in_json_string = false;
      while (i < n) {
        const char c = line[i];
        if (in_json_string) {
          if (c == '\\' && i + 1 < n) {
            tok.text.push_back(c);
            tok.text.push_back(line[i + 1]);
            i += 2;
            continue;
          }
          if (c == '"') in_json_string = false;
          tok.text.push_back(c);
          ++i;
          continue;
        }
        if (depth == 0 && is_space(c)) break;
        if (c == '\\' && i + 1 < n) {
          tok.text.push_back(line[i + 1]);
          i += 2;
          continue;
        }
        if (c == '"') {
          in_json_string = true;
          tok.text.push_back(c);
          ++i;
          continue;
        }
        if (c == '[' || c == '{') {
          ++depth;
          tok.text.push_back(c);
          ++i;
          continue;
        }
        if (c == ']' || c == '}') {
          --depth;
          if (depth < 0) throw ParseError("rpc_repl: unbalanced brackets (unexpected closing bracket) in command line");
          tok.text.push_back(c);
          ++i;
          continue;
        }
        tok.text.push_back(c);
        ++i;
      }
      if (in_json_string) throw ParseError("rpc_repl: unterminated string inside brackets in command line");
      if (depth != 0) throw ParseError("rpc_repl: unbalanced brackets in command line");
    }
    tokens.push_back(std::move(tok));
  }
  return tokens;
}

/// Converts one token to a `Value`: a forced-string (quoted) token is
/// always a `Value` string; otherwise the token is attempted as a JSON
/// literal, falling back to a plain string if it doesn't parse.
inline serial_rpc::Value token_to_value(const Token &tok) {
  if (tok.forced_string) return serial_rpc::Value(tok.text);
  json parsed = json::parse(tok.text, /*callback*/ nullptr, /*allow_exceptions*/ false);
  if (parsed.is_discarded()) return serial_rpc::Value(tok.text);
  return json_to_value(parsed);
}

} // namespace detail

/// Result of `parse_args()`: `command` is empty for a blank (or
/// whitespace-only) line.
struct ParsedCommand {
  std::string command;
  std::vector<serial_rpc::Value> args;
};

/// Tokenizes `line` (see `detail::tokenize()`) and converts it to a command
/// name plus a vector of `Value` arguments: the first token is always taken
/// as a literal string command name (never JSON-interpreted, quoted or
/// not); every later token is converted with `detail::token_to_value()`.
/// Throws ParseError on the same malformed input `tokenize()` does.
inline ParsedCommand parse_args(std::string_view line) {
  const std::vector<detail::Token> tokens = detail::tokenize(line);
  ParsedCommand out;
  if (tokens.empty()) return out;
  out.command = tokens.front().text;
  out.args.reserve(tokens.size() - 1);
  for (size_t i = 1; i < tokens.size(); ++i) out.args.push_back(detail::token_to_value(tokens[i]));
  return out;
}

} // namespace rpc_repl
