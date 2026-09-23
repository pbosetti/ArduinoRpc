/// \file serial_rpc.hpp
/// \brief Host-side, header-only MsgPack-RPC client (see docs/PLAN.md,
///   component 5): a dynamically-typed `Value`, and the `RPC<Port>` class
///   that drives a request/response/notification exchange with a device
///   running `serial_rpc::Server` (arduino/SerialRPC/src/serial_rpc/server.h)
///   over anything shaped like `SerialPort`.
///
/// C++20, header-only. Includes the shared wire-level headers straight from
/// the Arduino library folder (single source of truth for the codec and the
/// framing, per docs/PLAN.md's "Layout" section) plus `serialport.hpp`.
#pragma once

#include "serial_rpc/config.h"
#include "serial_rpc/framing.h"
#include "serial_rpc/msgpack_lite.h"
#include "serialport.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace serial_rpc {

// ===========================================================================
// Errors. Every exception this header throws derives from Error, which
// derives from std::runtime_error, so a plain `catch (const std::exception&)`
// always works.
// ===========================================================================

/// Base class for every exception thrown by this header.
class Error : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

/// Thrown by `Value::as<T>()` when the held alternative can't convert to
/// `T` (wrong wire type, or an integer/float that doesn't fit `T`'s range).
class TypeError : public Error {
public:
  using Error::Error;
};

/// Thrown by `RPC::call()`/`call_values()` when the response's error slot
/// was non-nil. `error_value()` holds the decoded error, usually (but not
/// necessarily) a string, exactly as the device sent it.
class RemoteError; // defined after Value, below.

/// Thrown when a call, `connect()`, or `poll()` runs out of time waiting
/// for a response or a "ready" line.
class TimeoutError : public Error {
public:
  using Error::Error;
};

/// Thrown on a malformed or out-of-protocol frame (bad msgpack shape,
/// oversized payload, unsupported msgpack type, nesting too deep).
class ProtocolError : public Error {
public:
  using Error::Error;
};

/// Thrown when the underlying `Port` reports an I/O failure (a negative
/// return from `write()`/`read_some()`, or a short write).
class IoError : public Error {
public:
  using Error::Error;
};

namespace detail {

template <class>
inline constexpr bool always_false_v = false;

template <class T>
struct is_std_vector : std::false_type {};
template <class T, class A>
struct is_std_vector<std::vector<T, A>> : std::true_type {};
template <class T>
inline constexpr bool is_std_vector_v = is_std_vector<T>::value;

/// Matches `std::map<std::string, V, ...>` specifically -- Value::as<T>()
/// only supports string-keyed maps, per docs/PLAN.md.
template <class T>
struct is_string_keyed_map : std::false_type {};
template <class V, class C, class A>
struct is_string_keyed_map<std::map<std::string, V, C, A>> : std::true_type {};
template <class T>
inline constexpr bool is_string_keyed_map_v = is_string_keyed_map<T>::value;

/// Formats a double compactly for Value::to_string(): %.17g round-trips
/// every IEEE-754 binary64 value while still printing short values (like
/// 3.5 or -2.25) without spurious trailing digits, since %g trims to the
/// precision actually needed.
inline std::string format_double(double v) {
  if (std::isnan(v)) return "nan";
  if (std::isinf(v)) return v > 0 ? "inf" : "-inf";
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  return std::string(buf);
}

/// Appends `s` to `out` as the body of a JSON double-quoted string (no
/// surrounding quotes), escaping the characters JSON requires.
inline void append_json_escaped(std::string &out, std::string_view s) {
  for (unsigned char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += static_cast<char>(c);
        }
    }
  }
}

} // namespace detail

// ===========================================================================
// Value: an owning, dynamically-typed MessagePack object.
// ===========================================================================

/// \brief A small owning dynamic type mirroring every shape MessagePack can
///   carry (docs/PLAN.md, component 5): nil, bool, a signed or unsigned
///   64-bit integer, a double, a string, a binary blob, an array, or an
///   *ordered* map.
///
/// Integers keep the wire's signed/unsigned distinction (`int64_t` vs
/// `uint64_t`) rather than collapsing to one, so a `uint64_t` too large for
/// `int64_t` (e.g. the top half of the uint64 range) round-trips exactly.
/// The map alternative is `vector<pair<Value,Value>>`, not `std::map`, so
/// key order -- and duplicate keys, if a peer sends them -- survive a
/// decode/encode round trip.
class Value {
public:
  using Bin = std::vector<uint8_t>;
  using Array = std::vector<Value>;
  using MapEntry = std::pair<Value, Value>;
  using Map = std::vector<MapEntry>;

  /// Default-constructs a nil value.
  Value() : _data(std::monostate{}) {}
  Value(std::nullptr_t) : _data(std::monostate{}) {} // NOLINT(*-explicit-constructor)
  Value(bool v) : _data(v) {}                        // NOLINT(*-explicit-constructor)
  Value(int64_t v) : _data(v) {}                      // NOLINT(*-explicit-constructor)
  Value(uint64_t v) : _data(v) {}                     // NOLINT(*-explicit-constructor)
  Value(double v) : _data(v) {}                       // NOLINT(*-explicit-constructor)
  Value(std::string v) : _data(std::move(v)) {}       // NOLINT(*-explicit-constructor)
  Value(std::string_view v) : _data(std::string(v)) {}// NOLINT(*-explicit-constructor)
  Value(const char *v) : _data(std::string(v)) {}     // NOLINT(*-explicit-constructor)
  Value(Bin v) : _data(std::move(v)) {}                // NOLINT(*-explicit-constructor)
  Value(Array v) : _data(std::move(v)) {}              // NOLINT(*-explicit-constructor)
  Value(Map v) : _data(std::move(v)) {}                // NOLINT(*-explicit-constructor)

  /// Any other fundamental integer type (int, long, unsigned, ...): routes
  /// to the int64_t or uint64_t alternative by T's own signedness. Kept
  /// separate from the int64_t/uint64_t constructors above (rather than one
  /// template covering every integral T) so that `Value(int64_t{...})` and
  /// `Value(uint64_t{...})` are exact, unambiguous matches instead of tying
  /// with this template.
  template <class T>
    requires(std::is_integral_v<T> && !std::is_same_v<T, bool> && !std::is_same_v<T, int64_t> &&
             !std::is_same_v<T, uint64_t>)
  Value(T v) { // NOLINT(*-explicit-constructor)
    if constexpr (std::is_signed_v<T>) {
      _data = static_cast<int64_t>(v);
    } else {
      _data = static_cast<uint64_t>(v);
    }
  }

  bool is_nil() const noexcept { return std::holds_alternative<std::monostate>(_data); }
  bool is_bool() const noexcept { return std::holds_alternative<bool>(_data); }
  bool is_int() const noexcept { return std::holds_alternative<int64_t>(_data); }
  bool is_uint() const noexcept { return std::holds_alternative<uint64_t>(_data); }
  bool is_integer() const noexcept { return is_int() || is_uint(); }
  bool is_double() const noexcept { return std::holds_alternative<double>(_data); }
  bool is_number() const noexcept { return is_integer() || is_double(); }
  bool is_string() const noexcept { return std::holds_alternative<std::string>(_data); }
  bool is_bin() const noexcept { return std::holds_alternative<Bin>(_data); }
  bool is_array() const noexcept { return std::holds_alternative<Array>(_data); }
  bool is_map() const noexcept { return std::holds_alternative<Map>(_data); }

  /// Converts to `T`, throwing TypeError on a wire-type mismatch or an
  /// out-of-range numeric conversion. Supported `T`: `Value` (identity),
  /// `bool`, any fundamental integer type (range-checked against the wire
  /// int64_t/uint64_t), any floating-point type, `std::string`, `Bin`
  /// (`std::vector<uint8_t>`), `std::vector<T>` (recursively, from an
  /// array) and `std::map<std::string, T>` (recursively, from a map, keys
  /// converted with `as<std::string>()`).
  template <class T>
  T as() const {
    if constexpr (std::is_same_v<T, Value>) {
      return *this;
    } else if constexpr (std::is_same_v<T, bool>) {
      if (const auto *p = std::get_if<bool>(&_data)) return *p;
      throw TypeError("serial_rpc::Value::as<bool>(): value is not a bool (" + to_string() + ")");
    } else if constexpr (std::is_same_v<T, std::string>) {
      if (const auto *p = std::get_if<std::string>(&_data)) return *p;
      throw TypeError("serial_rpc::Value::as<std::string>(): value is not a string (" + to_string() + ")");
    } else if constexpr (std::is_same_v<T, Bin>) {
      if (const auto *p = std::get_if<Bin>(&_data)) return *p;
      throw TypeError("serial_rpc::Value::as<Bin>(): value is not bin (" + to_string() + ")");
    } else if constexpr (std::is_integral_v<T>) {
      return as_integral_<T>();
    } else if constexpr (std::is_floating_point_v<T>) {
      return as_floating_<T>();
    } else if constexpr (detail::is_std_vector_v<T>) {
      using Elem = typename T::value_type;
      if (const auto *p = std::get_if<Array>(&_data)) {
        T out;
        out.reserve(p->size());
        for (const auto &e : *p) out.push_back(e.template as<Elem>());
        return out;
      }
      throw TypeError("serial_rpc::Value::as<vector<T>>(): value is not an array (" + to_string() + ")");
    } else if constexpr (detail::is_string_keyed_map_v<T>) {
      using Mapped = typename T::mapped_type;
      if (const auto *p = std::get_if<Map>(&_data)) {
        T out;
        for (const auto &kv : *p) out.emplace(kv.first.template as<std::string>(), kv.second.template as<Mapped>());
        return out;
      }
      throw TypeError("serial_rpc::Value::as<map<string,T>>(): value is not a map (" + to_string() + ")");
    } else {
      static_assert(detail::always_false_v<T>,
                    "serial_rpc::Value::as<T>(): unsupported T; supported types are Value, bool, the fundamental "
                    "integer types, floating-point types, std::string, Bin (std::vector<uint8_t>), std::vector<T> "
                    "and std::map<std::string,T>");
    }
  }

  /// Compact JSON-like rendering: strings quoted and escaped, bin shown as
  /// `<bin N bytes>`, everything else in the obvious JSON-ish shape (a map
  /// prints as `{k: v, ...}` in its stored, not sorted, order).
  std::string to_string() const {
    std::string out;
    append_to_string_(out);
    return out;
  }

  /// Decodes one MessagePack object from `r`, recursively for array/map
  /// elements. Throws ProtocolError on malformed input, an unsupported wire
  /// type (ext), or nesting deeper than `kMaxDepth`.
  static Value decode(msgpack::Reader &r) { return decode_(r, 0); }

  /// Encodes this value into `w` (nil for `w.pack_nil()`, and so on),
  /// recursively for array/map elements.
  void encode(msgpack::Writer &w) const {
    std::visit(
        [&w](const auto &v) {
          using U = std::decay_t<decltype(v)>;
          if constexpr (std::is_same_v<U, std::monostate>) {
            w.pack_nil();
          } else if constexpr (std::is_same_v<U, bool>) {
            w.pack(v);
          } else if constexpr (std::is_same_v<U, int64_t>) {
            w.pack(v);
          } else if constexpr (std::is_same_v<U, uint64_t>) {
            w.pack(v);
          } else if constexpr (std::is_same_v<U, double>) {
            w.pack(v);
          } else if constexpr (std::is_same_v<U, std::string>) {
            w.pack_str(v.data(), v.size());
          } else if constexpr (std::is_same_v<U, Bin>) {
            w.pack_bin(v.data(), v.size());
          } else if constexpr (std::is_same_v<U, Array>) {
            w.pack_array(v.size());
            for (const Value &e : v) e.encode(w);
          } else if constexpr (std::is_same_v<U, Map>) {
            w.pack_map(v.size());
            for (const MapEntry &kv : v) {
              kv.first.encode(w);
              kv.second.encode(w);
            }
          }
        },
        _data);
  }

  static constexpr int kMaxDepth = 32;

private:
  static Value decode_(msgpack::Reader &r, int depth) {
    if (r.error()) throw ProtocolError("serial_rpc::Value::decode(): reader is already in an error state");
    if (depth > kMaxDepth) throw ProtocolError("serial_rpc::Value::decode(): msgpack nesting exceeds kMaxDepth");

    switch (r.type()) {
      case msgpack::Reader::Type::nil: {
        r.read_nil();
        check_(r);
        return Value();
      }
      case msgpack::Reader::Type::boolean: {
        bool v = false;
        r.read(v);
        check_(r);
        return Value(v);
      }
      case msgpack::Reader::Type::int_: {
        int64_t v = 0;
        r.read(v);
        check_(r);
        return Value(v);
      }
      case msgpack::Reader::Type::uint_: {
        uint64_t v = 0;
        r.read(v);
        check_(r);
        return Value(v);
      }
      case msgpack::Reader::Type::float_: {
        double v = 0;
        r.read(v);
        check_(r);
        return Value(v);
      }
      case msgpack::Reader::Type::str: {
        const char *p = nullptr;
        size_t len = 0;
        r.read_str(p, len);
        check_(r);
        return Value(std::string(p, len));
      }
      case msgpack::Reader::Type::bin: {
        const uint8_t *p = nullptr;
        size_t len = 0;
        r.read_bin(p, len);
        check_(r);
        return Value(Bin(p, p + len));
      }
      case msgpack::Reader::Type::array: {
        size_t n = 0;
        r.read_array(n);
        check_(r);
        Array arr;
        arr.reserve(n);
        for (size_t i = 0; i < n; ++i) arr.push_back(decode_(r, depth + 1));
        return Value(std::move(arr));
      }
      case msgpack::Reader::Type::map: {
        size_t n = 0;
        r.read_map(n);
        check_(r);
        Map m;
        m.reserve(n);
        for (size_t i = 0; i < n; ++i) {
          Value k = decode_(r, depth + 1);
          Value v = decode_(r, depth + 1);
          m.emplace_back(std::move(k), std::move(v));
        }
        return Value(std::move(m));
      }
      case msgpack::Reader::Type::ext:
      case msgpack::Reader::Type::invalid:
      default:
        throw ProtocolError("serial_rpc::Value::decode(): unsupported or invalid msgpack type");
    }
  }

  static void check_(const msgpack::Reader &r) {
    if (r.error()) throw ProtocolError("serial_rpc::Value::decode(): malformed msgpack input");
  }

  void append_to_string_(std::string &out) const {
    std::visit(
        [&out](const auto &v) {
          using U = std::decay_t<decltype(v)>;
          if constexpr (std::is_same_v<U, std::monostate>) {
            out += "nil";
          } else if constexpr (std::is_same_v<U, bool>) {
            out += v ? "true" : "false";
          } else if constexpr (std::is_same_v<U, int64_t>) {
            out += std::to_string(v);
          } else if constexpr (std::is_same_v<U, uint64_t>) {
            out += std::to_string(v);
          } else if constexpr (std::is_same_v<U, double>) {
            out += detail::format_double(v);
          } else if constexpr (std::is_same_v<U, std::string>) {
            out += '"';
            detail::append_json_escaped(out, v);
            out += '"';
          } else if constexpr (std::is_same_v<U, Bin>) {
            out += "<bin " + std::to_string(v.size()) + " bytes>";
          } else if constexpr (std::is_same_v<U, Array>) {
            out += '[';
            for (size_t i = 0; i < v.size(); ++i) {
              if (i) out += ", ";
              v[i].append_to_string_(out);
            }
            out += ']';
          } else if constexpr (std::is_same_v<U, Map>) {
            out += '{';
            for (size_t i = 0; i < v.size(); ++i) {
              if (i) out += ", ";
              v[i].first.append_to_string_(out);
              out += ": ";
              v[i].second.append_to_string_(out);
            }
            out += '}';
          }
        },
        _data);
  }

  template <class T>
  T as_integral_() const {
    if (const auto *p = std::get_if<int64_t>(&_data)) {
      const int64_t v = *p;
      if constexpr (std::is_unsigned_v<T>) {
        if (v < 0 || static_cast<uint64_t>(v) > static_cast<uint64_t>(std::numeric_limits<T>::max())) {
          throw TypeError("serial_rpc::Value::as<T>(): integer " + std::to_string(v) + " does not fit the target "
                           "unsigned type");
        }
      } else {
        if (v < static_cast<int64_t>(std::numeric_limits<T>::min()) ||
            v > static_cast<int64_t>(std::numeric_limits<T>::max())) {
          throw TypeError("serial_rpc::Value::as<T>(): integer " + std::to_string(v) + " does not fit the target "
                           "type");
        }
      }
      return static_cast<T>(v);
    }
    if (const auto *p = std::get_if<uint64_t>(&_data)) {
      const uint64_t v = *p;
      if constexpr (std::is_signed_v<T>) {
        if (v > static_cast<uint64_t>(std::numeric_limits<T>::max())) {
          throw TypeError("serial_rpc::Value::as<T>(): integer " + std::to_string(v) + " does not fit the target "
                           "type");
        }
      } else {
        if (v > static_cast<uint64_t>(std::numeric_limits<T>::max())) {
          throw TypeError("serial_rpc::Value::as<T>(): integer " + std::to_string(v) + " does not fit the target "
                           "unsigned type");
        }
      }
      return static_cast<T>(v);
    }
    throw TypeError("serial_rpc::Value::as<T>(): value is not an integer (" + to_string() + ")");
  }

  template <class T>
  T as_floating_() const {
    if (const auto *p = std::get_if<double>(&_data)) return static_cast<T>(*p);
    if (const auto *p = std::get_if<int64_t>(&_data)) return static_cast<T>(*p);
    if (const auto *p = std::get_if<uint64_t>(&_data)) return static_cast<T>(*p);
    throw TypeError("serial_rpc::Value::as<T>(): value is not a number (" + to_string() + ")");
  }

  std::variant<std::monostate, bool, int64_t, uint64_t, double, std::string, Bin, Array, Map> _data;
};

inline std::ostream &operator<<(std::ostream &os, const Value &v) { return os << v.to_string(); }

/// See the forward declaration above: defined here, now that Value is
/// complete.
class RemoteError : public Error {
public:
  RemoteError(std::string method, Value error_value)
      : Error(build_message_(method, error_value)), _method(std::move(method)), _error(std::move(error_value)) {}

  /// The remote method that was called.
  const std::string &method() const noexcept { return _method; }
  /// The decoded contents of the response's error slot, exactly as the
  /// device sent it (usually, but not necessarily, a string).
  const Value &error_value() const noexcept { return _error; }

private:
  static std::string build_message_(const std::string &method, const Value &err) {
    std::string msg = "serial_rpc: remote error calling '" + method + "': ";
    msg += err.is_string() ? err.as<std::string>() : err.to_string();
    return msg;
  }

  std::string _method;
  Value _error;
};

namespace detail {

/// Converts one `call()`/`notify()` argument to a Value. Overload/template
/// set covering every type docs/PLAN.md lists for the variadic call sites:
/// ints, floats, bool, strings (std::string / string_view / const char*),
/// Value itself (passthrough), nullptr_t, std::vector<T> and
/// std::map<std::string,T>.
template <class T>
Value to_value(const T &v) {
  using U = std::decay_t<T>;
  if constexpr (std::is_same_v<U, Value>) {
    return v;
  } else if constexpr (std::is_same_v<U, std::nullptr_t>) {
    return Value();
  } else if constexpr (std::is_same_v<U, bool>) {
    return Value(v);
  } else if constexpr (std::is_same_v<U, std::string> || std::is_same_v<U, std::string_view>) {
    return Value(std::string(v));
  } else if constexpr (std::is_convertible_v<U, const char *>) {
    return Value(std::string(static_cast<const char *>(v)));
  } else if constexpr (std::is_integral_v<U>) {
    if constexpr (std::is_signed_v<U>) return Value(static_cast<int64_t>(v));
    else return Value(static_cast<uint64_t>(v));
  } else if constexpr (std::is_floating_point_v<U>) {
    return Value(static_cast<double>(v));
  } else if constexpr (is_std_vector_v<U>) {
    Value::Array arr;
    arr.reserve(v.size());
    for (const auto &e : v) arr.push_back(to_value(e));
    return Value(std::move(arr));
  } else if constexpr (is_string_keyed_map_v<U>) {
    Value::Map m;
    m.reserve(v.size());
    for (const auto &kv : v) m.emplace_back(Value(kv.first), to_value(kv.second));
    return Value(std::move(m));
  } else {
    static_assert(always_false_v<U>, "serial_rpc: unsupported call()/notify() argument type; supported types are "
                  "Value, nullptr_t, bool, integers, floating point, strings, std::vector<T> and "
                  "std::map<std::string,T>");
  }
}

/// Saves `port`'s current timeout and restores it on scope exit, after
/// narrowing it to `budget` for the guard's lifetime. Used to bound each
/// `read_some()` call to the remaining time left in a call/connect/poll
/// deadline, since the Port duck type honours its *own* timeout() inside
/// read_some() (docs/PLAN.md component 5's Port requirements).
template <class Port>
class TimeoutGuard {
public:
  TimeoutGuard(Port &port, std::chrono::milliseconds budget) : _port(port), _saved(port.timeout()) {
    _port.set_timeout(budget);
  }
  ~TimeoutGuard() { _port.set_timeout(_saved); }
  TimeoutGuard(const TimeoutGuard &) = delete;
  TimeoutGuard &operator=(const TimeoutGuard &) = delete;

private:
  Port &_port;
  decltype(std::declval<Port &>().timeout()) _saved;
};

/// Minimal RAII "run this on scope exit" helper, used where a member
/// function needs to guarantee some cleanup (like clearing a pending-call
/// msgid) on every exit path, including an exception thrown mid-loop.
/// Holding a `std::function`-free template lambda type (rather than a
/// local struct with a raw `self` pointer) sidesteps any question about
/// whether a class local to a member function shares that function's
/// access to private members: a lambda's closure type unambiguously does.
template <class F>
class ScopeExit {
public:
  explicit ScopeExit(F f) : _f(std::move(f)) {}
  ~ScopeExit() { _f(); }
  ScopeExit(const ScopeExit &) = delete;
  ScopeExit &operator=(const ScopeExit &) = delete;

private:
  F _f;
};

/// Writes `len` bytes at `data` through whichever of Port's two accepted
/// write() shapes it actually has: `write(const char*, size_t)` or
/// `write(std::span<const std::byte>)` (docs/PLAN.md component 5's Port
/// requirements list both as acceptable).
template <class Port>
int port_write(Port &port, const uint8_t *data, size_t len) {
  if constexpr (requires { port.write(reinterpret_cast<const char *>(data), len); }) {
    return static_cast<int>(port.write(reinterpret_cast<const char *>(data), len));
  } else {
    return static_cast<int>(port.write(std::span<const std::byte>(reinterpret_cast<const std::byte *>(data), len)));
  }
}

} // namespace detail

/// A bound method's name and, when the device reports one (docs/PLAN.md
/// component 7's `rpc.list` signature strings), its signature; empty when
/// absent. `RPC::list()` accepts both `rpc.list` shapes a device may return
/// -- a plain array of name strings, or an array of `[name, signature]`
/// pairs -- normalizing either into this.
struct MethodInfo {
  std::string name;
  std::string signature;
};

/// Normalizes an `rpc.list` result `Value` (an array of strings, or an
/// array of `[name, signature]` pairs) into a `vector<MethodInfo>`. A free
/// function, not a private RPC member, so it can be unit-tested directly
/// against a hand-built Value without a live device on either side of it.
inline std::vector<MethodInfo> parse_rpc_list(const Value &v) {
  if (!v.is_array()) throw ProtocolError("serial_rpc: rpc.list did not return an array");
  std::vector<MethodInfo> out;
  for (const Value &item : v.as<std::vector<Value>>()) {
    MethodInfo mi;
    if (item.is_string()) {
      mi.name = item.as<std::string>();
    } else if (item.is_array()) {
      const auto arr = item.as<std::vector<Value>>();
      if (arr.empty() || !arr[0].is_string()) {
        throw ProtocolError("serial_rpc: rpc.list element is an array but its first element isn't a name string");
      }
      mi.name = arr[0].as<std::string>();
      if (arr.size() >= 2 && arr[1].is_string()) mi.signature = arr[1].as<std::string>();
    } else {
      throw ProtocolError("serial_rpc: rpc.list element is neither a string nor a [name, signature] array");
    }
    out.push_back(std::move(mi));
  }
  return out;
}

/// Maximum number of `rpc.list` page requests `collect_rpc_list_pages()`
/// will make before giving up: a guard against a misbehaving device that
/// never signals the end, not a limit on how many methods can be listed
/// (each page can hold many entries).
inline constexpr int kMaxListPages = 256;

/// Core pagination loop behind `RPC::list()`, factored out as a free
/// function (taking a page-fetching callback rather than an RPC) so it can
/// be unit-tested directly against a scripted sequence of pages, without a
/// live device on either end.
///
/// Device-side `rpc.list [start]` (docs/PLAN.md component 7) returns as
/// many `[name, signature]` entries as fit starting at `start`, and an
/// empty array marks the end. This calls `fetch_page(start)` with
/// `start` = the number of distinct method names collected so far,
/// accumulating any entries not already seen, until either:
///  - a page comes back empty (the normal, paging-aware end-of-list), or
///  - a page adds nothing new -- which also covers a device that ignores
///    `start` entirely and just keeps re-sending its one full list (so the
///    first call already returns everything, and every call after that
///    "adds nothing new" since every name in it is already known), or
///  - `kMaxListPages` calls have been made.
inline std::vector<MethodInfo> collect_rpc_list_pages(
    const std::function<std::vector<MethodInfo>(int64_t start)> &fetch_page) {
  std::vector<MethodInfo> out;
  for (int iter = 0; iter < kMaxListPages; ++iter) {
    const std::vector<MethodInfo> page = fetch_page(static_cast<int64_t>(out.size()));
    if (page.empty()) break;

    size_t added = 0;
    for (const auto &mi : page) {
      const bool already_have =
          std::any_of(out.begin(), out.end(), [&](const MethodInfo &e) { return e.name == mi.name; });
      if (!already_have) {
        out.push_back(mi);
        ++added;
      }
    }
    if (added == 0) break;
  }
  return out;
}

// ===========================================================================
// RPC<Port>: the host-side client.
// ===========================================================================

/// \brief Host-side MsgPack-RPC client (docs/PLAN.md, component 5): sends
///   requests/notifications to a `serial_rpc::Server` over `Port` and reads
///   responses back through a shared `Demux`, dispatching logs,
///   notifications and text lines as they arrive along the way.
///
/// `Port` is duck-typed, not required to inherit from anything -- the
/// subset used is `write(std::span<const std::byte>)` or
/// `write(const char*, size_t)`, `read_some(char*, size_t)` (honouring its
/// own configured timeout), `set_timeout`/`timeout`, `set_dtr`, and
/// `flush_input`. `SerialRPCClient` below is the alias for the real
/// `SerialPort`; tests instead instantiate `RPC<LoopbackPort>` against an
/// in-process mock.
///
/// **Not thread-safe.** Exactly one thread must own an `RPC` instance (and
/// the `Port` it wraps) for its whole lifetime, same as `SerialPort` itself.
template <class Port = SerialPort>
class RPC {
public:
  using clock = std::chrono::steady_clock;

  /// Tunables. `timeout` bounds every `call()`. `max_frame` sizes this
  /// object's own outgoing payload/frame buffers (how large a request or
  /// notification this side can *send*); it does not resize the incoming
  /// `Demux`, which is fixed generously large at compile time -- see
  /// `HostDemux` below.
  struct Options {
    std::chrono::milliseconds timeout{1000};
    size_t max_frame = 1024;
  };

  struct Stats {
    uint64_t calls = 0;
    uint64_t notifications_sent = 0;
    uint64_t notifications_recv = 0;
    uint64_t logs_recv = 0;
    uint64_t text_recv = 0;
    uint64_t stray_responses = 0;
    uint64_t timeouts = 0;
    uint64_t malformed_frames = 0;
  };

  explicit RPC(Port &port, Options opts = {})
      : _port(port), _opts(opts), _demux(std::make_unique<HostDemux>()), _tx_payload(opts.max_frame),
        _tx_frame(frame_max_size(opts.max_frame)) {
    _on_text = [](std::string_view line) { std::cerr << line << "\n"; };
  }

  ~RPC() noexcept {
    if (_attached) {
      try {
        disconnect();
      } catch (...) {
        // Destructors don't throw; best-effort detach only.
      }
    }
  }

  RPC(const RPC &) = delete;
  RPC &operator=(const RPC &) = delete;

  // --- calls ---------------------------------------------------------------

  /// Calls `method` with `args...` (packed via `detail::to_value`) and
  /// returns the result, either as a generic `Value` (the default) or
  /// converted with `Value::as<R>()` when `R` is given explicitly, e.g.
  /// `rpc.call<int>("get_temp")`. Throws RemoteError on an error response,
  /// TimeoutError if no response arrives within Options::timeout.
  template <class R = Value, class... A>
  R call(std::string_view method, const A &...args) {
    Value result = call_values(method, pack_args_(args...));
    if constexpr (std::is_same_v<R, Value>) {
      return result;
    } else {
      return result.template as<R>();
    }
  }

  /// Same as `call<R>(...)`, spelled with `R` first, per docs/PLAN.md's
  /// alternate naming suggestion; a thin wrapper over `call<R>()`.
  template <class R, class... A>
  R call_as(std::string_view method, const A &...args) {
    return call<R>(method, args...);
  }

  /// The dynamic form `call()`/`call<R>()` are built on: takes an
  /// already-packed argument array. The REPL (docs/PLAN.md component 7)
  /// needs exactly this, since it builds `Value` args from parsed JSON
  /// tokens at runtime.
  Value call_values(std::string_view method, const std::vector<Value> &args) {
    const uint32_t msgid = next_msgid_();
    send_request_(msgid, method, args);
    _pending_msgid = msgid;
    _pending_response_ready = false;
    detail::ScopeExit guard([this] { _pending_msgid.reset(); });
    ++_stats.calls;

    const auto start = clock::now();
    const auto deadline = start + _opts.timeout;
    while (true) {
      const auto now = clock::now();
      if (now >= deadline) {
        ++_stats.timeouts;
        throw TimeoutError("serial_rpc: timed out waiting for a response to '" + std::string(method) + "'");
      }
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
      const auto attempt = std::min(remaining, std::chrono::milliseconds(100));
      pump_bytes_(attempt);
      if (_pending_response_ready) {
        _last_rtt = std::chrono::duration_cast<std::chrono::microseconds>(clock::now() - start);
        if (_pending_response_is_error) {
          Value err = std::move(_pending_response_value);
          throw RemoteError(std::string(method), std::move(err));
        }
        return std::move(_pending_response_value);
      }
    }
  }

  /// Fire-and-forget: `[2, method, [args...]]`, no reply expected.
  template <class... A>
  void notify(std::string_view method, const A &...args) {
    notify_values(method, pack_args_(args...));
  }

  /// Dynamic form of `notify()`.
  void notify_values(std::string_view method, const std::vector<Value> &args) {
    msgpack::Writer w(_tx_payload.data(), _tx_payload.size());
    w.pack_array(3);
    w.pack(2);
    w.pack_str(method.data(), method.size());
    w.pack_array(args.size());
    for (const Value &a : args) a.encode(w);
    if (w.overflow()) throw ProtocolError("serial_rpc: notification payload exceeds max_frame");
    write_frame_(w.data(), w.size());
    ++_stats.notifications_sent;
  }

  /// Sends a raw text line (a trailing '\n' is appended) to the device's
  /// `on_text` hook -- not framed, so it works exactly like typing into a
  /// terminal talking to the sketch, whether attached or not.
  void send_text(std::string_view line) {
    std::string buf(line);
    buf.push_back('\n');
    const int n = detail::port_write(_port, reinterpret_cast<const uint8_t *>(buf.data()), buf.size());
    if (n < 0 || static_cast<size_t>(n) != buf.size()) throw IoError("serial_rpc: send_text() write failed");
  }

  // --- inbound callbacks -----------------------------------------------------

  /// Framed `"log"` notifications (only sent while the device is attached).
  void on_log(std::function<void(int level, std::string_view line)> cb) { _on_log = std::move(cb); }

  /// Raw, unframed text lines (default: printed to stderr). This also
  /// receives a detached device's plain-text `rpc.log` lines.
  void on_text(std::function<void(std::string_view line)> cb) { _on_text = std::move(cb); }

  /// Registers/replaces a handler for one notification method's args.
  void on(std::string method, std::function<void(const Value &args)> cb) { _handlers[std::move(method)] = std::move(cb); }

  /// Catch-all for every notification (in addition to any specific `on()`
  /// handler for the same method) -- what the REPL's generic event display
  /// (docs/PLAN.md component 7) is built on.
  void on_any_notification(std::function<void(std::string_view method, const Value &args)> cb) {
    _on_any = std::move(cb);
  }

  // --- polling / connection lifecycle -----------------------------------

  /// Processes all input available within `max_wait` (at least one
  /// non-blocking pass, even for the default `max_wait = 0`), dispatching
  /// logs/notifications/text as it goes. Returns the number of such events
  /// dispatched. Meant to be called from an idle loop when no call is in
  /// flight (`call()` pumps the same input for itself while waiting).
  size_t poll(std::chrono::milliseconds max_wait = std::chrono::milliseconds(0)) {
    const uint64_t before = events_total_();
    const auto deadline = clock::now() + max_wait;
    do {
      const auto now = clock::now();
      const auto attempt = now < deadline ? std::min(std::chrono::duration_cast<std::chrono::milliseconds>(
                                                           deadline - now),
                                                       std::chrono::milliseconds(50))
                                           : std::chrono::milliseconds(0);
      pump_bytes_(attempt);
    } while (clock::now() < deadline);
    return static_cast<size_t>(events_total_() - before);
  }

  /// Waits (up to `wait`) for the device to prove it's alive -- either a
  /// text line containing "ready", or a successful `rpc.ping`, retried
  /// every 250 ms -- optionally pulsing DTR first to reset the board, then
  /// sends `rpc.attach`. Throws TimeoutError if neither happens in time.
  void connect(std::chrono::milliseconds wait = std::chrono::milliseconds(3000), bool reset = false) {
    _demux->reset();
    _port.flush_input();

    if (reset) {
      _port.set_dtr(false);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      _port.set_dtr(true);
    }

    const auto deadline = clock::now() + wait;
    bool saw_ready = false;
    const auto prev_on_text = _on_text;
    _on_text = [&saw_ready, &prev_on_text](std::string_view line) {
      if (line.find("ready") != std::string_view::npos) saw_ready = true;
      if (prev_on_text) prev_on_text(line);
    };

    bool got_reply = false;
    while (!saw_ready && !got_reply) {
      const auto now = clock::now();
      if (now >= deadline) break;
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
      const auto budget = std::min(remaining, std::chrono::milliseconds(250));
      if (try_request_("rpc.ping", budget).has_value()) got_reply = true;
    }
    _on_text = prev_on_text;

    if (!saw_ready && !got_reply) {
      throw TimeoutError("serial_rpc: connect() timed out waiting for a 'ready' line or an rpc.ping reply");
    }

    call("rpc.attach");
    _attached = true;
  }

  /// Sends `rpc.detach` (a notification: no reply expected) if attached.
  /// Also called, with errors swallowed, by the destructor.
  void disconnect() {
    if (_attached) {
      notify("rpc.detach");
      _attached = false;
    }
  }

  bool attached() const noexcept { return _attached; }

  /// Calls `rpc.list`, paging through results with a `start` argument
  /// (docs/PLAN.md component 7) until the device signals the end, and
  /// normalizes each page -- see `collect_rpc_list_pages()` and
  /// `parse_rpc_list()`. Also works against a device that doesn't page at
  /// all (ignores `start`, always returns everything in one reply): the
  /// currently-checked-out device build in this repo is exactly that, and
  /// `collect_rpc_list_pages()` stops cleanly after one extra, now-fully-
  /// redundant call in that case rather than looping or duplicating.
  std::vector<MethodInfo> list() {
    return collect_rpc_list_pages([this](int64_t start) { return parse_rpc_list(call("rpc.list", start)); });
  }

  /// Calls `rpc.ping`, returning the device's protocol version.
  int ping() { return call<int>("rpc.ping"); }

  std::chrono::microseconds last_rtt() const noexcept { return _last_rtt; }
  const Stats &stats() const noexcept { return _stats; }

  Port &port() noexcept { return _port; }

private:
  /// Generous, fixed-at-compile-time Demux sizing for the host side (see
  /// docs/PLAN.md's implementation note for this component): large enough
  /// for any reasonable device BufSize's worst-case COBS-encoded frame plus
  /// a generous text-line buffer, regardless of Options::max_frame. At
  /// ~5 KB, this is heap-allocated via unique_ptr rather than embedded
  /// directly in RPC, so RPC objects stay cheap to construct/move around.
  using HostDemux = Demux<4096, 1024>;

  uint32_t next_msgid_() { return _next_msgid++; }

  uint64_t events_total_() const {
    return _stats.text_recv + _stats.notifications_recv + _stats.logs_recv + _stats.stray_responses;
  }

  template <class... A>
  static std::vector<Value> pack_args_(const A &...args) {
    std::vector<Value> v;
    v.reserve(sizeof...(A));
    (v.push_back(detail::to_value(args)), ...);
    return v;
  }

  void send_request_(uint32_t msgid, std::string_view method, const std::vector<Value> &args) {
    msgpack::Writer w(_tx_payload.data(), _tx_payload.size());
    w.pack_array(4);
    w.pack(0);
    w.pack(msgid);
    w.pack_str(method.data(), method.size());
    w.pack_array(args.size());
    for (const Value &a : args) a.encode(w);
    if (w.overflow()) throw ProtocolError("serial_rpc: request payload exceeds max_frame");
    write_frame_(w.data(), w.size());
  }

  void write_frame_(const uint8_t *payload, size_t len) {
    const size_t n = frame_encode(payload, len, _tx_frame.data(), _tx_frame.size());
    if (n == 0) throw ProtocolError("serial_rpc: frame_encode() overflow (payload too large for max_frame)");
    const int written = detail::port_write(_port, _tx_frame.data(), n);
    if (written < 0 || static_cast<size_t>(written) != n) throw IoError("serial_rpc: write() failed or short write");
  }

  /// Sends a request and pumps input for up to `budget`, returning the
  /// result on a successful reply or `nullopt` on a timeout. Used by
  /// `connect()` for its retried `rpc.ping` probes, where a per-attempt
  /// timeout (not Options::timeout, and not a thrown TimeoutError) is
  /// wanted. Still throws RemoteError if the device actually answers with
  /// an error.
  std::optional<Value> try_request_(std::string_view method, std::chrono::milliseconds budget) {
    const uint32_t msgid = next_msgid_();
    send_request_(msgid, method, {});
    _pending_msgid = msgid;
    _pending_response_ready = false;
    detail::ScopeExit guard([this] { _pending_msgid.reset(); });

    const auto deadline = clock::now() + budget;
    while (true) {
      const auto now = clock::now();
      if (now >= deadline) return std::nullopt;
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
      const auto attempt = std::min(remaining, std::chrono::milliseconds(50));
      pump_bytes_(attempt);
      if (_pending_response_ready) {
        _pending_response_ready = false;
        if (_pending_response_is_error) {
          Value err = std::move(_pending_response_value);
          throw RemoteError(std::string(method), std::move(err));
        }
        return std::move(_pending_response_value);
      }
    }
  }

  /// One bounded read: narrows the port's timeout to `budget` for the
  /// duration of a single `read_some()` call, feeds every byte read into
  /// the Demux, and dispatches whatever events fall out. Returns the
  /// number of bytes read (0 on a timeout with nothing available).
  int pump_bytes_(std::chrono::milliseconds budget) {
    detail::TimeoutGuard<Port> guard(_port, budget);
    char buf[256];
    const int n = _port.read_some(buf, sizeof(buf));
    if (n < 0) throw IoError("serial_rpc: read_some() failed");
    for (int i = 0; i < n; ++i) {
      const DemuxEvent ev = _demux->feed(static_cast<uint8_t>(buf[i]));
      if (ev == DemuxEvent::frame) {
        handle_frame_(_demux->data(), _demux->size());
      } else if (ev == DemuxEvent::text_line) {
        ++_stats.text_recv;
        const std::string_view line(reinterpret_cast<const char *>(_demux->data()), _demux->size());
        if (_on_text) _on_text(line);
      }
    }
    return n;
  }

  void handle_frame_(const uint8_t *data, size_t len) {
    msgpack::Reader r(data, len);
    size_t arr_n = 0;
    r.read_array(arr_n);
    if (r.error()) {
      ++_stats.malformed_frames;
      return;
    }
    int type = 0;
    r.read(type);
    if (r.error()) {
      ++_stats.malformed_frames;
      return;
    }

    if (type == 1) { // response: [1, msgid, error|nil, result|nil]
      if (arr_n != 4) {
        ++_stats.malformed_frames;
        return;
      }
      uint32_t msgid = 0;
      r.read(msgid);
      if (r.error()) {
        ++_stats.malformed_frames;
        return;
      }
      const bool is_error = r.type() != msgpack::Reader::Type::nil;
      Value err_value;
      if (is_error) {
        err_value = Value::decode(r);
      } else {
        r.read_nil();
      }
      if (r.error()) {
        ++_stats.malformed_frames;
        return;
      }
      Value result = Value::decode(r);
      if (r.error()) {
        ++_stats.malformed_frames;
        return;
      }

      if (_pending_msgid.has_value() && msgid == *_pending_msgid) {
        _pending_response_ready = true;
        _pending_response_is_error = is_error;
        _pending_response_value = is_error ? std::move(err_value) : std::move(result);
      } else {
        ++_stats.stray_responses;
      }
    } else if (type == 2) { // notification: [2, method, params]
      if (arr_n != 3) {
        ++_stats.malformed_frames;
        return;
      }
      const char *mptr = nullptr;
      size_t mlen = 0;
      r.read_str(mptr, mlen);
      if (r.error()) {
        ++_stats.malformed_frames;
        return;
      }
      const std::string method(mptr, mlen);
      Value args = Value::decode(r);
      if (r.error()) {
        ++_stats.malformed_frames;
        return;
      }

      if (method == "log") {
        ++_stats.logs_recv;
        dispatch_log_(args);
      } else {
        ++_stats.notifications_recv;
        dispatch_notification_(method, args);
      }
    } else {
      ++_stats.malformed_frames; // a device never sends a request (type 0)
    }
  }

  void dispatch_log_(const Value &args) {
    if (!_on_log) return;
    try {
      const auto arr = args.as<std::vector<Value>>();
      if (arr.size() < 2) return;
      const int level = arr[0].as<int>();
      const std::string line = arr[1].as<std::string>();
      _on_log(level, line);
    } catch (const TypeError &) {
      // Malformed "log" notification shape; nothing sensible to dispatch.
    }
  }

  void dispatch_notification_(const std::string &method, const Value &args) {
    if (const auto it = _handlers.find(method); it != _handlers.end()) it->second(args);
    if (_on_any) _on_any(method, args);
  }

  Port &_port;
  Options _opts;
  std::unique_ptr<HostDemux> _demux;
  std::vector<uint8_t> _tx_payload;
  std::vector<uint8_t> _tx_frame;

  uint32_t _next_msgid = 1;
  std::optional<uint32_t> _pending_msgid;
  bool _pending_response_ready = false;
  bool _pending_response_is_error = false;
  Value _pending_response_value;

  bool _attached = false;
  std::chrono::microseconds _last_rtt{0};
  Stats _stats;

  std::function<void(int, std::string_view)> _on_log;
  std::function<void(std::string_view)> _on_text;
  std::map<std::string, std::function<void(const Value &)>> _handlers;
  std::function<void(std::string_view, const Value &)> _on_any;
};

/// Convenience alias for the common case: `serial_rpc::SerialRPCClient rpc(port);`.
using SerialRPCClient = RPC<SerialPort>;

} // namespace serial_rpc

/// `fmt::formatter<serial_rpc::Value>`, defined only if `<fmt/format.h>` (or
/// any other fmt header defining FMT_VERSION) was already included before
/// this header -- so this header never pulls fmt in itself, per
/// docs/PLAN.md's "don't make fmt a hard dependency".
#if defined(FMT_VERSION)
template <>
struct fmt::formatter<serial_rpc::Value> : fmt::formatter<std::string> {
  template <class FormatContext>
  auto format(const serial_rpc::Value &v, FormatContext &ctx) const {
    return fmt::formatter<std::string>::format(v.to_string(), ctx);
  }
};
#endif
