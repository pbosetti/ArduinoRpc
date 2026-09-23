// SPDX-License-Identifier: Apache-2.0
/// \file msgpack_lite.h
/// \brief Minimal, dependency-free MessagePack codec shared by the Arduino
///   device library and the host RPC client.
///
/// Hard constraints (see docs/PLAN.md, component 1):
///  - Must compile as C++11 with avr-gcc 7.3: no STL headers (only the
///    freestanding C headers <stdint.h>, <string.h>, <stddef.h>), no
///    exceptions, no heap allocation, no <type_traits>.
///  - Must also compile warning-free as C++20 with Clang on the host.
///  - No endianness assumptions: every multi-byte value is encoded and
///    decoded byte-by-byte in MessagePack's big-endian wire order.
///
/// `Writer` serializes into a caller-owned buffer and never throws; a
/// buffer that is too small sets a sticky overflow() flag instead.
/// `Reader` is a cursor over a caller-owned buffer; malformed or truncated
/// input sets a sticky error() flag instead of throwing, and every read is
/// atomic: on failure the output parameter is left untouched and the
/// cursor position is restored to where the call started.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace serial_rpc {
namespace msgpack {

namespace detail {

/// MessagePack format byte constants (see the spec's format table).
enum FormatByte : uint8_t {
  kPositiveFixintMax = 0x7f,
  kFixmapPrefix = 0x80,
  kFixarrayPrefix = 0x90,
  kFixstrPrefix = 0xa0,
  kNil = 0xc0,
  kReserved = 0xc1, // never used by the spec
  kFalse = 0xc2,
  kTrue = 0xc3,
  kBin8 = 0xc4,
  kBin16 = 0xc5,
  kBin32 = 0xc6,
  kExt8 = 0xc7,
  kExt16 = 0xc8,
  kExt32 = 0xc9,
  kFloat32 = 0xca,
  kFloat64 = 0xcb,
  kUint8 = 0xcc,
  kUint16 = 0xcd,
  kUint32 = 0xce,
  kUint64 = 0xcf,
  kInt8 = 0xd0,
  kInt16 = 0xd1,
  kInt32 = 0xd2,
  kInt64 = 0xd3,
  kFixext1 = 0xd4,
  kFixext2 = 0xd5,
  kFixext4 = 0xd6,
  kFixext8 = 0xd7,
  kFixext16 = 0xd8,
  kStr8 = 0xd9,
  kStr16 = 0xda,
  kStr32 = 0xdb,
  kArray16 = 0xdc,
  kArray32 = 0xdd,
  kMap16 = 0xde,
  kMap32 = 0xdf,
  kNegativeFixintMin = 0xe0
};

/// The largest value size_t can hold, as a uint64_t, computed without
/// relying on the SIZE_MAX macro being defined by every <stdint.h>.
inline uint64_t size_t_max_as_u64() {
  return static_cast<uint64_t>(static_cast<size_t>(-1));
}

// Portable maximum-value constants for the ten fundamental integer types,
// computed with the standard "unsigned wraps to all-ones, signed max is
// that shifted right by one" bit trick instead of pulling them from
// <limits.h>. This sidesteps a real gap found on avr-gcc 7.3 / avr-libc:
// its <limits.h> does not define LLONG_MAX/ULLONG_MAX even though the
// compiler fully supports `long long` under -std=gnu++11.
inline unsigned char max_uchar() { return static_cast<unsigned char>(-1); }
inline unsigned short max_ushort() { return static_cast<unsigned short>(-1); }
inline unsigned int max_uint() { return static_cast<unsigned int>(-1); }
inline unsigned long max_ulong() { return static_cast<unsigned long>(-1); }
inline unsigned long long max_ullong() { return static_cast<unsigned long long>(-1); }
inline signed char max_schar() { return static_cast<signed char>(max_uchar() >> 1); }
inline short max_short() { return static_cast<short>(max_ushort() >> 1); }
inline int max_int() { return static_cast<int>(max_uint() >> 1); }
inline long max_long() { return static_cast<long>(max_ulong() >> 1); }
inline long long max_llong() { return static_cast<long long>(max_ullong() >> 1); }

/// Narrows an IEEE-754 binary64 bit pattern to the nearest binary32 bit
/// pattern. Needed whenever a MessagePack float64 has to end up in a
/// 32-bit `float` (always) or in an AVR `double` (which shares float's
/// 32-bit representation under avr-gcc). This is a simple round-toward-zero
/// (truncating) narrowing conversion, not a fully IEEE-correct
/// round-to-nearest-even one: subnormal binary64 inputs and magnitudes too
/// small for binary32 flush to zero, magnitudes too large saturate to
/// +-infinity, and the mantissa is truncated rather than rounded. This is
/// an accepted, documented precision trade-off for a microcontroller codec.
inline uint32_t narrow_f64_bits_to_f32_bits(uint64_t bits64) {
  const uint32_t sign = static_cast<uint32_t>((bits64 >> 63) & 0x1u) << 31;
  const uint32_t biased_exp64 = static_cast<uint32_t>((bits64 >> 52) & 0x7ffu);
  const uint64_t mantissa64 = bits64 & 0xfffffffffffffull; // low 52 bits

  if (biased_exp64 == 0x7ffu) { // inf or nan
    const uint32_t mantissa32 = mantissa64 ? 0x400000u : 0u; // keep quiet-NaN-ness
    return sign | 0x7f800000u | mantissa32;
  }
  if (biased_exp64 == 0) { // zero or subnormal: too small to matter for float32
    return sign;
  }

  const int32_t exponent = static_cast<int32_t>(biased_exp64) - 1023 + 127; // rebias
  if (exponent >= 0xff) {
    return sign | 0x7f800000u; // overflow -> +-infinity
  }
  if (exponent <= 0) {
    return sign; // underflow -> +-0
  }

  const uint32_t mantissa32 = static_cast<uint32_t>(mantissa64 >> 29); // 52 -> 23 bits
  return sign | (static_cast<uint32_t>(exponent) << 23) | mantissa32;
}

} // namespace detail

/// \brief Serializes MessagePack objects into a caller-provided buffer.
///
/// Never allocates and never throws. A write that does not fit in the
/// buffer is dropped and the sticky overflow() flag is set; size() always
/// reflects the number of bytes actually written, never more than the
/// capacity passed to the constructor.
class Writer {
public:
  /// \param buf Destination buffer, owned by the caller.
  /// \param cap Capacity of `buf` in bytes.
  Writer(uint8_t *buf, size_t cap) : _buf(buf), _cap(cap), _size(0), _overflow(false) {}

  /// Number of bytes written so far (always <= capacity).
  size_t size() const { return _size; }
  /// Pointer to the start of the written bytes.
  const uint8_t *data() const { return _buf; }
  /// True once any write did not fit in the buffer.
  bool overflow() const { return _overflow; }
  /// Clears size() and overflow(); does not touch buffer contents.
  void reset() {
    _size = 0;
    _overflow = false;
  }

  /// Writes a nil object.
  void pack_nil() { put_byte_(detail::kNil); }

  /// Writes a boolean object.
  void pack(bool v) { put_byte_(v ? detail::kTrue : detail::kFalse); }

  // Concrete (non-template) overloads for every fundamental integer type.
  // stdint.h typedefs (int8_t, uint32_t, ...) always alias one of these on
  // any real platform, so they resolve to exactly one overload here with
  // no ambiguity and no need for <type_traits>.
  void pack(signed char v) { pack_int_(v); }
  void pack(short v) { pack_int_(v); }
  void pack(int v) { pack_int_(v); }
  void pack(long v) { pack_int_(v); }
  void pack(long long v) { pack_int_(v); }
  void pack(unsigned char v) { pack_uint_(v); }
  void pack(unsigned short v) { pack_uint_(v); }
  void pack(unsigned int v) { pack_uint_(v); }
  void pack(unsigned long v) { pack_uint_(v); }
  void pack(unsigned long long v) { pack_uint_(v); }

  /// Writes an IEEE-754 binary32 float.
  void pack(float v) {
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    put_byte_(detail::kFloat32);
    put_u32_(bits);
  }

  /// Writes a double. On hosts where `sizeof(double) == 8` this emits a
  /// MessagePack float64. On AVR (avr-gcc), where `double` and `float`
  /// share the same 32-bit representation, it emits a float32 instead --
  /// there is no extra precision to gain from a float64 there, and
  /// treating the two as interchangeable avoids ever memcpy-ing 8 bytes
  /// out of a 4-byte object. The choice is made with a preprocessor `#if`
  /// (not a runtime `if` or a template) specifically so that the branch
  /// that does not apply to the current target is never compiled at all,
  /// not merely dead at runtime.
  void pack(double v) {
#if defined(__SIZEOF_DOUBLE__) && defined(__SIZEOF_FLOAT__) && (__SIZEOF_DOUBLE__ == __SIZEOF_FLOAT__)
    pack(static_cast<float>(v));
#else
    uint64_t bits;
    memcpy(&bits, &v, sizeof(bits));
    put_byte_(detail::kFloat64);
    put_u64_(bits);
#endif
  }

  // On a platform where size_t is 16 bits wide (AVR), no size_t value can
  // ever reach 65536, so a "< 65536" branch guarding the *16 vs *32
  // encodings would be tautologically true and (rightly) warned about by
  // -Wtype-limits -- not something a cast can honestly silence, since the
  // value truly can never get there. Rather than fight the warning, the
  // *_32 encoding (and the comparison that would pick it) is simply not
  // compiled at all on such platforms: the final "else" always means *16
  // there. Platforms with a wider size_t (32- or 64-bit) keep the full
  // fixed/*8/*16/*32 ladder, where the comparison is real.
#if !defined(__SIZEOF_SIZE_T__) || (__SIZEOF_SIZE_T__ > 2)
#define SERIAL_RPC_MSGPACK_HAS_32BIT_HEADERS_ 1
#else
#define SERIAL_RPC_MSGPACK_HAS_32BIT_HEADERS_ 0
#endif

  /// Writes a string object, choosing fixstr/str8/str16(/str32) by length.
  void pack_str(const char *s, size_t len) {
    if (len < 32) {
      put_byte_(static_cast<uint8_t>(detail::kFixstrPrefix | len));
    } else if (len < 256) {
      put_byte_(detail::kStr8);
      put_byte_(static_cast<uint8_t>(len));
#if SERIAL_RPC_MSGPACK_HAS_32BIT_HEADERS_
    } else if (len < 65536) {
      put_byte_(detail::kStr16);
      put_u16_(static_cast<uint16_t>(len));
    } else {
      put_byte_(detail::kStr32);
      put_u32_(static_cast<uint32_t>(len));
    }
#else
    } else {
      put_byte_(detail::kStr16);
      put_u16_(static_cast<uint16_t>(len));
    }
#endif
    put_bytes_(reinterpret_cast<const uint8_t *>(s), len);
  }

  /// Writes a NUL-terminated C string as a MessagePack string object.
  void pack(const char *s) { pack_str(s, strlen(s)); }

  /// Writes a binary blob, choosing bin8/bin16(/bin32) by length.
  void pack_bin(const uint8_t *bytes, size_t len) {
    if (len < 256) {
      put_byte_(detail::kBin8);
      put_byte_(static_cast<uint8_t>(len));
#if SERIAL_RPC_MSGPACK_HAS_32BIT_HEADERS_
    } else if (len < 65536) {
      put_byte_(detail::kBin16);
      put_u16_(static_cast<uint16_t>(len));
    } else {
      put_byte_(detail::kBin32);
      put_u32_(static_cast<uint32_t>(len));
    }
#else
    } else {
      put_byte_(detail::kBin16);
      put_u16_(static_cast<uint16_t>(len));
    }
#endif
    put_bytes_(bytes, len);
  }

  /// Writes an array header announcing `n` following elements.
  void pack_array(size_t n) {
    if (n < 16) {
      put_byte_(static_cast<uint8_t>(detail::kFixarrayPrefix | n));
#if SERIAL_RPC_MSGPACK_HAS_32BIT_HEADERS_
    } else if (n < 65536) {
      put_byte_(detail::kArray16);
      put_u16_(static_cast<uint16_t>(n));
    } else {
      put_byte_(detail::kArray32);
      put_u32_(static_cast<uint32_t>(n));
    }
#else
    } else {
      put_byte_(detail::kArray16);
      put_u16_(static_cast<uint16_t>(n));
    }
#endif
  }

  /// Writes a map header announcing `n` following key/value pairs.
  void pack_map(size_t n) {
    if (n < 16) {
      put_byte_(static_cast<uint8_t>(detail::kFixmapPrefix | n));
#if SERIAL_RPC_MSGPACK_HAS_32BIT_HEADERS_
    } else if (n < 65536) {
      put_byte_(detail::kMap16);
      put_u16_(static_cast<uint16_t>(n));
    } else {
      put_byte_(detail::kMap32);
      put_u32_(static_cast<uint32_t>(n));
    }
#else
    } else {
      put_byte_(detail::kMap16);
      put_u16_(static_cast<uint16_t>(n));
    }
#endif
  }

#undef SERIAL_RPC_MSGPACK_HAS_32BIT_HEADERS_

  /// Packs zero objects; base case for the variadic pack_all() below.
  void pack_all() {}

  /// Packs each argument in turn with pack(), e.g.
  /// `w.pack_all(13, true, "hi")` is `w.pack(13); w.pack(true); w.pack("hi");`.
  template <typename First, typename... Rest>
  void pack_all(const First &first, const Rest &...rest) {
    pack(first);
    pack_all(rest...);
  }

private:
  void put_byte_(uint8_t b) {
    if (_size < _cap) {
      _buf[_size] = b;
      ++_size;
    } else {
      _overflow = true;
    }
  }

  void put_bytes_(const uint8_t *p, size_t n) {
    for (size_t i = 0; i < n; ++i) put_byte_(p[i]);
  }

  void put_u16_(uint16_t v) {
    put_byte_(static_cast<uint8_t>((v >> 8) & 0xffu));
    put_byte_(static_cast<uint8_t>(v & 0xffu));
  }

  void put_u32_(uint32_t v) {
    put_byte_(static_cast<uint8_t>((v >> 24) & 0xffu));
    put_byte_(static_cast<uint8_t>((v >> 16) & 0xffu));
    put_byte_(static_cast<uint8_t>((v >> 8) & 0xffu));
    put_byte_(static_cast<uint8_t>(v & 0xffu));
  }

  void put_u64_(uint64_t v) {
    put_u32_(static_cast<uint32_t>((v >> 32) & 0xffffffffull));
    put_u32_(static_cast<uint32_t>(v & 0xffffffffull));
  }

  /// Picks the smallest unsigned encoding (fixint/uint8/16/32/64).
  void pack_uint_(uint64_t v) {
    if (v <= detail::kPositiveFixintMax) {
      put_byte_(static_cast<uint8_t>(v));
    } else if (v <= 0xffull) {
      put_byte_(detail::kUint8);
      put_byte_(static_cast<uint8_t>(v));
    } else if (v <= 0xffffull) {
      put_byte_(detail::kUint16);
      put_u16_(static_cast<uint16_t>(v));
    } else if (v <= 0xffffffffull) {
      put_byte_(detail::kUint32);
      put_u32_(static_cast<uint32_t>(v));
    } else {
      put_byte_(detail::kUint64);
      put_u64_(v);
    }
  }

  /// Picks the smallest signed encoding (fixint/int8/16/32/64), falling
  /// back to pack_uint_() for non-negative values so they get the smallest
  /// *unsigned* encoding instead (msgpack has no separate "positive int8").
  void pack_int_(int64_t v) {
    if (v >= 0) {
      pack_uint_(static_cast<uint64_t>(v));
      return;
    }
    if (v >= -32) {
      put_byte_(static_cast<uint8_t>(v)); // negative fixint, two's complement bit pattern
    } else if (v >= -128) {
      put_byte_(detail::kInt8);
      put_byte_(static_cast<uint8_t>(v));
    } else if (v >= -32768) {
      put_byte_(detail::kInt16);
      put_u16_(static_cast<uint16_t>(v));
    } else if (v >= -2147483648ll) {
      put_byte_(detail::kInt32);
      put_u32_(static_cast<uint32_t>(v));
    } else {
      put_byte_(detail::kInt64);
      put_u64_(static_cast<uint64_t>(v));
    }
  }

  uint8_t *_buf;
  size_t _cap;
  size_t _size;
  bool _overflow;
};

/// \brief A cursor over a caller-owned buffer that decodes MessagePack
///   objects out of it.
///
/// Never allocates and never throws. Every read is atomic: on success it
/// fully consumes the object and writes the output parameter; on failure
/// it sets the sticky error() flag, restores the cursor to where the call
/// started, and leaves the output parameter untouched. Once error() is
/// true, further reads are effectively no-ops (they immediately fail the
/// same way).
class Reader {
public:
  /// Coarse classification of the next object, from type().
  enum class Type { nil, boolean, int_, uint_, float_, str, bin, array, map, ext, invalid };

  /// \param buf Source buffer, owned by the caller, valid for the Reader's lifetime.
  /// \param size Number of valid bytes in `buf`.
  Reader(const uint8_t *buf, size_t size) : _buf(buf), _size(size), _pos(0), _error(false) {}

  /// Bytes not yet consumed.
  size_t remaining() const { return _size - _pos; }
  /// True once any read has failed (malformed or truncated input).
  bool error() const { return _error; }

  /// Peeks (does not consume) the next object's format byte and classifies
  /// it. Positive fixint/uint8/16/32/64 report as uint_; negative
  /// fixint/int8/16/32/64 report as int_.
  Type type() const {
    if (_error || remaining() == 0) return Type::invalid;
    const uint8_t b = _buf[_pos];
    if (b <= detail::kPositiveFixintMax) return Type::uint_;
    if (b >= detail::kNegativeFixintMin) return Type::int_;
    if ((b & 0xf0u) == detail::kFixmapPrefix) return Type::map;
    if ((b & 0xf0u) == detail::kFixarrayPrefix) return Type::array;
    if ((b & 0xe0u) == detail::kFixstrPrefix) return Type::str;
    switch (b) {
      case detail::kNil: return Type::nil;
      case detail::kFalse:
      case detail::kTrue: return Type::boolean;
      case detail::kBin8:
      case detail::kBin16:
      case detail::kBin32: return Type::bin;
      case detail::kExt8:
      case detail::kExt16:
      case detail::kExt32:
      case detail::kFixext1:
      case detail::kFixext2:
      case detail::kFixext4:
      case detail::kFixext8:
      case detail::kFixext16: return Type::ext;
      case detail::kFloat32:
      case detail::kFloat64: return Type::float_;
      case detail::kUint8:
      case detail::kUint16:
      case detail::kUint32:
      case detail::kUint64: return Type::uint_;
      case detail::kInt8:
      case detail::kInt16:
      case detail::kInt32:
      case detail::kInt64: return Type::int_;
      case detail::kStr8:
      case detail::kStr16:
      case detail::kStr32: return Type::str;
      case detail::kArray16:
      case detail::kArray32: return Type::array;
      case detail::kMap16:
      case detail::kMap32: return Type::map;
      default: return Type::invalid; // 0xc1 is reserved
    }
  }

  /// Reads a nil object.
  void read_nil() {
    const size_t saved = _pos;
    uint8_t b;
    if (!get_byte_(b) || b != detail::kNil) fail_(saved);
  }

  /// Reads a boolean object.
  void read(bool &out) {
    const size_t saved = _pos;
    uint8_t b;
    if (!get_byte_(b)) { fail_(saved); return; }
    if (b == detail::kTrue) { out = true; return; }
    if (b == detail::kFalse) { out = false; return; }
    fail_(saved);
  }

  // Integer reads: strict, int/uint wire families only (never float),
  // range-checked against the target type's own limits.
  void read(signed char &out) { read_signed_(out, detail::max_schar()); }
  void read(short &out) { read_signed_(out, detail::max_short()); }
  void read(int &out) { read_signed_(out, detail::max_int()); }
  void read(long &out) { read_signed_(out, detail::max_long()); }
  void read(long long &out) { read_signed_(out, detail::max_llong()); }
  void read(unsigned char &out) { read_unsigned_(out, detail::max_uchar()); }
  void read(unsigned short &out) { read_unsigned_(out, detail::max_ushort()); }
  void read(unsigned int &out) { read_unsigned_(out, detail::max_uint()); }
  void read(unsigned long &out) { read_unsigned_(out, detail::max_ulong()); }
  void read(unsigned long long &out) { read_unsigned_(out, detail::max_ullong()); }

  /// Reads a float32, float64 (narrowed) or int/uint (converted) object.
  void read(float &out) {
    const size_t saved = _pos;
    if (remaining() == 0) { fail_(saved); return; }
    const uint8_t b = _buf[_pos];
    if (b == detail::kFloat32 || b == detail::kFloat64) {
      ++_pos;
      if (b == detail::kFloat32) {
        uint32_t bits;
        if (!get_u32_(bits)) { fail_(saved); return; }
        memcpy(&out, &bits, sizeof(out));
      } else {
        uint64_t bits;
        if (!get_u64_(bits)) { fail_(saved); return; }
        const uint32_t f32bits = detail::narrow_f64_bits_to_f32_bits(bits);
        memcpy(&out, &f32bits, sizeof(out));
      }
      return;
    }
    uint64_t mag;
    bool neg;
    if (!read_raw_magnitude_(mag, neg)) return;
    out = neg ? -static_cast<float>(mag) : static_cast<float>(mag);
  }

  /// Reads a float64, float32 (widened) or int/uint (converted) object.
  /// On AVR, where `double` shares float's 32-bit representation, an
  /// incoming float64 is narrowed the same way read(float&) does.
  void read(double &out) {
    const size_t saved = _pos;
    if (remaining() == 0) { fail_(saved); return; }
    const uint8_t b = _buf[_pos];
    if (b == detail::kFloat32 || b == detail::kFloat64) {
      ++_pos;
      if (b == detail::kFloat32) {
        uint32_t bits;
        if (!get_u32_(bits)) { fail_(saved); return; }
        float f;
        memcpy(&f, &bits, sizeof(f));
        out = static_cast<double>(f);
      } else {
        uint64_t bits;
        if (!get_u64_(bits)) { fail_(saved); return; }
#if defined(__SIZEOF_DOUBLE__) && defined(__SIZEOF_FLOAT__) && (__SIZEOF_DOUBLE__ == __SIZEOF_FLOAT__)
        const uint32_t f32bits = detail::narrow_f64_bits_to_f32_bits(bits);
        memcpy(&out, &f32bits, sizeof(out));
#else
        memcpy(&out, &bits, sizeof(out));
#endif
      }
      return;
    }
    uint64_t mag;
    bool neg;
    if (!read_raw_magnitude_(mag, neg)) return;
    out = neg ? -static_cast<double>(mag) : static_cast<double>(mag);
  }

  /// Zero-copy string read: points `ptr` into the source buffer and sets
  /// `len`. `ptr` is only valid as long as the source buffer is.
  void read_str(const char *&ptr, size_t &len) {
    const size_t saved = _pos;
    size_t n;
    if (!read_str_header_(n)) return;
    if (remaining() < n) { fail_(saved); return; }
    ptr = reinterpret_cast<const char *>(_buf + _pos);
    len = n;
    _pos += n;
  }

  /// Copying string read: copies into `dst` and NUL-terminates. Fails
  /// (error set, `dst` untouched) if the string plus terminator would not
  /// fit in `cap` bytes.
  void read_str(char *dst, size_t cap) {
    const size_t saved = _pos;
    size_t n;
    if (!read_str_header_(n)) return;
    if (cap == 0 || n >= cap) { fail_(saved); return; }
    if (remaining() < n) { fail_(saved); return; }
    memcpy(dst, _buf + _pos, n);
    dst[n] = '\0';
    _pos += n;
  }

  /// Zero-copy binary read: points `ptr` into the source buffer and sets `len`.
  void read_bin(const uint8_t *&ptr, size_t &len) {
    const size_t saved = _pos;
    size_t n;
    if (!read_bin_header_(n)) return;
    if (remaining() < n) { fail_(saved); return; }
    ptr = _buf + _pos;
    len = n;
    _pos += n;
  }

  /// Reads an array header, setting `n` to the number of following elements.
  void read_array(size_t &n) {
    const size_t saved = _pos;
    uint8_t b;
    if (!get_byte_(b)) { fail_(saved); return; }
    if ((b & 0xf0u) == detail::kFixarrayPrefix) { n = b & 0x0fu; return; }
    if (b == detail::kArray16) {
      uint16_t v;
      if (!get_u16_(v)) { fail_(saved); return; }
      n = v;
      return;
    }
    if (b == detail::kArray32) {
      uint32_t v;
      if (!get_u32_(v)) { fail_(saved); return; }
      if (static_cast<uint64_t>(v) > detail::size_t_max_as_u64()) { fail_(saved); return; }
      n = static_cast<size_t>(v);
      return;
    }
    fail_(saved);
  }

  /// Reads a map header, setting `n` to the number of following key/value pairs.
  void read_map(size_t &n) {
    const size_t saved = _pos;
    uint8_t b;
    if (!get_byte_(b)) { fail_(saved); return; }
    if ((b & 0xf0u) == detail::kFixmapPrefix) { n = b & 0x0fu; return; }
    if (b == detail::kMap16) {
      uint16_t v;
      if (!get_u16_(v)) { fail_(saved); return; }
      n = v;
      return;
    }
    if (b == detail::kMap32) {
      uint32_t v;
      if (!get_u32_(v)) { fail_(saved); return; }
      if (static_cast<uint64_t>(v) > detail::size_t_max_as_u64()) { fail_(saved); return; }
      n = static_cast<size_t>(v);
      return;
    }
    fail_(saved);
  }

  /// Skips one object of any type, including nested arrays/maps and ext
  /// types (ext is supported here only, not via a dedicated read()).
  /// Uses an explicit iterative "objects still owed" counter rather than
  /// recursion, so it is safe on AVR's small stack regardless of nesting
  /// depth: skipping an array/map of n children simply adds n (or 2n for
  /// a map's key/value pairs) more objects to the flat count still to be
  /// skipped, in place of the one container object just consumed.
  void skip() {
    if (_error) return;
    const size_t saved = _pos;
    size_t remaining_objects = 1;
    while (remaining_objects > 0) {
      if (remaining() == 0) { fail_(saved); return; }
      const uint8_t b = _buf[_pos];

      if (b <= detail::kPositiveFixintMax || b >= detail::kNegativeFixintMin) {
        ++_pos;
        --remaining_objects;
        continue;
      }
      if ((b & 0xf0u) == detail::kFixmapPrefix) {
        const size_t n = b & 0x0fu;
        ++_pos;
        remaining_objects += 2 * n;
        --remaining_objects;
        continue;
      }
      if ((b & 0xf0u) == detail::kFixarrayPrefix) {
        const size_t n = b & 0x0fu;
        ++_pos;
        remaining_objects += n;
        --remaining_objects;
        continue;
      }
      if ((b & 0xe0u) == detail::kFixstrPrefix) {
        const size_t n = b & 0x1fu;
        if (remaining() < 1 + n) { fail_(saved); return; }
        _pos += 1 + n;
        --remaining_objects;
        continue;
      }

      switch (b) {
        case detail::kNil:
        case detail::kFalse:
        case detail::kTrue:
          ++_pos;
          --remaining_objects;
          continue;
        case detail::kUint8:
        case detail::kInt8:
          if (!skip_fixed_(2)) { fail_(saved); return; }
          --remaining_objects;
          continue;
        case detail::kUint16:
        case detail::kInt16:
          if (!skip_fixed_(3)) { fail_(saved); return; }
          --remaining_objects;
          continue;
        case detail::kUint32:
        case detail::kInt32:
        case detail::kFloat32:
          if (!skip_fixed_(5)) { fail_(saved); return; }
          --remaining_objects;
          continue;
        case detail::kUint64:
        case detail::kInt64:
        case detail::kFloat64:
          if (!skip_fixed_(9)) { fail_(saved); return; }
          --remaining_objects;
          continue;
        case detail::kStr8:
        case detail::kBin8: {
          if (remaining() < 2) { fail_(saved); return; }
          const size_t n = _buf[_pos + 1];
          if (!skip_fixed_(2 + n)) { fail_(saved); return; }
          --remaining_objects;
          continue;
        }
        case detail::kStr16:
        case detail::kBin16: {
          if (remaining() < 3) { fail_(saved); return; }
          const size_t n = (static_cast<size_t>(_buf[_pos + 1]) << 8) | _buf[_pos + 2];
          if (!skip_fixed_(3 + n)) { fail_(saved); return; }
          --remaining_objects;
          continue;
        }
        case detail::kStr32:
        case detail::kBin32: {
          if (remaining() < 5) { fail_(saved); return; }
          const uint32_t n = read_be_u32_at_(_pos + 1);
          if (static_cast<uint64_t>(n) > detail::size_t_max_as_u64() - 5) { fail_(saved); return; }
          if (!skip_fixed_(5 + static_cast<size_t>(n))) { fail_(saved); return; }
          --remaining_objects;
          continue;
        }
        case detail::kArray16: {
          if (remaining() < 3) { fail_(saved); return; }
          const size_t n = (static_cast<size_t>(_buf[_pos + 1]) << 8) | _buf[_pos + 2];
          _pos += 3;
          remaining_objects += n;
          --remaining_objects;
          continue;
        }
        case detail::kArray32: {
          if (remaining() < 5) { fail_(saved); return; }
          const uint32_t n = read_be_u32_at_(_pos + 1);
          _pos += 5;
          remaining_objects += static_cast<size_t>(n);
          --remaining_objects;
          continue;
        }
        case detail::kMap16: {
          if (remaining() < 3) { fail_(saved); return; }
          const size_t n = (static_cast<size_t>(_buf[_pos + 1]) << 8) | _buf[_pos + 2];
          _pos += 3;
          remaining_objects += 2 * n;
          --remaining_objects;
          continue;
        }
        case detail::kMap32: {
          if (remaining() < 5) { fail_(saved); return; }
          const uint32_t n = read_be_u32_at_(_pos + 1);
          _pos += 5;
          remaining_objects += 2 * static_cast<size_t>(n);
          --remaining_objects;
          continue;
        }
        case detail::kFixext1:
        case detail::kFixext2:
        case detail::kFixext4:
        case detail::kFixext8:
        case detail::kFixext16: {
          static const uint8_t kLen[5] = {1, 2, 4, 8, 16};
          const size_t idx = static_cast<size_t>(b - detail::kFixext1);
          if (!skip_fixed_(2 + kLen[idx])) { fail_(saved); return; } // format + type byte + payload
          --remaining_objects;
          continue;
        }
        case detail::kExt8: {
          if (remaining() < 2) { fail_(saved); return; }
          const size_t n = _buf[_pos + 1];
          if (!skip_fixed_(3 + n)) { fail_(saved); return; } // format + len + type + payload
          --remaining_objects;
          continue;
        }
        case detail::kExt16: {
          if (remaining() < 3) { fail_(saved); return; }
          const size_t n = (static_cast<size_t>(_buf[_pos + 1]) << 8) | _buf[_pos + 2];
          if (!skip_fixed_(4 + n)) { fail_(saved); return; }
          --remaining_objects;
          continue;
        }
        case detail::kExt32: {
          if (remaining() < 5) { fail_(saved); return; }
          const uint32_t n = read_be_u32_at_(_pos + 1);
          if (static_cast<uint64_t>(n) > detail::size_t_max_as_u64() - 6) { fail_(saved); return; }
          if (!skip_fixed_(6 + static_cast<size_t>(n))) { fail_(saved); return; }
          --remaining_objects;
          continue;
        }
        default:
          fail_(saved);
          return; // 0xc1 is reserved / not a valid object
      }
    }
  }

private:
  bool get_byte_(uint8_t &out) {
    if (remaining() == 0) { _error = true; return false; }
    out = _buf[_pos];
    ++_pos;
    return true;
  }

  bool get_bytes_(uint8_t *dst, size_t n) {
    if (remaining() < n) { _error = true; return false; }
    memcpy(dst, _buf + _pos, n);
    _pos += n;
    return true;
  }

  bool get_u16_(uint16_t &out) {
    uint8_t b[2];
    if (!get_bytes_(b, 2)) return false;
    out = static_cast<uint16_t>((static_cast<uint16_t>(b[0]) << 8) | b[1]);
    return true;
  }

  bool get_u32_(uint32_t &out) {
    uint8_t b[4];
    if (!get_bytes_(b, 4)) return false;
    out = (static_cast<uint32_t>(b[0]) << 24) | (static_cast<uint32_t>(b[1]) << 16) |
          (static_cast<uint32_t>(b[2]) << 8) | static_cast<uint32_t>(b[3]);
    return true;
  }

  bool get_u64_(uint64_t &out) {
    uint32_t hi, lo;
    if (!get_u32_(hi) || !get_u32_(lo)) return false;
    out = (static_cast<uint64_t>(hi) << 32) | lo;
    return true;
  }

  /// Reads a big-endian uint32_t at a known-in-bounds offset without
  /// advancing `_pos`; used by skip() after it has already range-checked.
  uint32_t read_be_u32_at_(size_t off) const {
    return (static_cast<uint32_t>(_buf[off]) << 24) | (static_cast<uint32_t>(_buf[off + 1]) << 16) |
           (static_cast<uint32_t>(_buf[off + 2]) << 8) | static_cast<uint32_t>(_buf[off + 3]);
  }

  /// Advances `_pos` by exactly `n` bytes if that many remain.
  bool skip_fixed_(size_t n) {
    if (remaining() < n) return false;
    _pos += n;
    return true;
  }

  void fail_(size_t saved_pos) {
    _error = true;
    _pos = saved_pos;
  }

  /// Decodes the next object as an int/uint-family value only (never
  /// float) into a magnitude + sign pair. `magnitude` is the absolute
  /// value, so it can represent INT64_MIN's magnitude (2^63) or any
  /// UINT64_MAX value without overflow.
  bool read_raw_magnitude_(uint64_t &magnitude, bool &is_negative) {
    const size_t saved = _pos;
    uint8_t b;
    if (!get_byte_(b)) { fail_(saved); return false; }
    if (b <= detail::kPositiveFixintMax) {
      magnitude = b;
      is_negative = false;
      return true;
    }
    if (b >= detail::kNegativeFixintMin) {
      const int8_t sv = static_cast<int8_t>(b);
      is_negative = true;
      magnitude = static_cast<uint64_t>(-static_cast<int64_t>(sv));
      return true;
    }
    switch (b) {
      case detail::kUint8: {
        uint8_t v;
        if (!get_byte_(v)) { fail_(saved); return false; }
        magnitude = v;
        is_negative = false;
        return true;
      }
      case detail::kUint16: {
        uint16_t v;
        if (!get_u16_(v)) { fail_(saved); return false; }
        magnitude = v;
        is_negative = false;
        return true;
      }
      case detail::kUint32: {
        uint32_t v;
        if (!get_u32_(v)) { fail_(saved); return false; }
        magnitude = v;
        is_negative = false;
        return true;
      }
      case detail::kUint64: {
        uint64_t v;
        if (!get_u64_(v)) { fail_(saved); return false; }
        magnitude = v;
        is_negative = false;
        return true;
      }
      case detail::kInt8: {
        uint8_t v;
        if (!get_byte_(v)) { fail_(saved); return false; }
        const int8_t sv = static_cast<int8_t>(v);
        is_negative = sv < 0;
        magnitude = is_negative ? static_cast<uint64_t>(-static_cast<int64_t>(sv)) : static_cast<uint64_t>(sv);
        return true;
      }
      case detail::kInt16: {
        uint16_t v;
        if (!get_u16_(v)) { fail_(saved); return false; }
        const int16_t sv = static_cast<int16_t>(v);
        is_negative = sv < 0;
        magnitude = is_negative ? static_cast<uint64_t>(-static_cast<int64_t>(sv)) : static_cast<uint64_t>(sv);
        return true;
      }
      case detail::kInt32: {
        uint32_t v;
        if (!get_u32_(v)) { fail_(saved); return false; }
        const int32_t sv = static_cast<int32_t>(v);
        is_negative = sv < 0;
        magnitude = is_negative ? static_cast<uint64_t>(-static_cast<int64_t>(sv)) : static_cast<uint64_t>(sv);
        return true;
      }
      case detail::kInt64: {
        uint64_t v;
        if (!get_u64_(v)) { fail_(saved); return false; }
        const int64_t sv = static_cast<int64_t>(v);
        is_negative = sv < 0;
        // -sv would overflow for sv == INT64_MIN; compute the magnitude
        // via unsigned arithmetic instead (two's complement negation).
        magnitude = is_negative ? (~static_cast<uint64_t>(sv) + 1u) : static_cast<uint64_t>(sv);
        return true;
      }
      default:
        fail_(saved);
        return false;
    }
  }

  /// Shared implementation for every signed read(T&) overload. `max_val`
  /// is T's own maximum; T's minimum is derived as `-max_val - 1`, which
  /// holds for every standard two's-complement signed integer type.
  template <typename T>
  void read_signed_(T &out, long long max_val) {
    const size_t saved = _pos;
    uint64_t mag;
    bool neg;
    if (!read_raw_magnitude_(mag, neg)) return;
    if (!neg) {
      if (mag > static_cast<uint64_t>(max_val)) { fail_(saved); return; }
      out = static_cast<T>(mag);
      return;
    }
    const uint64_t max_neg_mag = static_cast<uint64_t>(max_val) + 1u;
    if (mag > max_neg_mag) { fail_(saved); return; }
    if (mag == max_neg_mag) {
      out = static_cast<T>(-max_val - 1); // T's minimum, computed without overflow
    } else {
      out = static_cast<T>(-static_cast<long long>(mag));
    }
  }

  /// Shared implementation for every unsigned read(T&) overload.
  template <typename T>
  void read_unsigned_(T &out, unsigned long long max_val) {
    const size_t saved = _pos;
    uint64_t mag;
    bool neg;
    if (!read_raw_magnitude_(mag, neg)) return;
    if (neg) { fail_(saved); return; } // a negative source can never fit an unsigned target
    if (mag > static_cast<uint64_t>(max_val)) { fail_(saved); return; }
    out = static_cast<T>(mag);
  }

  bool read_str_header_(size_t &n) {
    const size_t saved = _pos;
    uint8_t b;
    if (!get_byte_(b)) { fail_(saved); return false; }
    if ((b & 0xe0u) == detail::kFixstrPrefix) { n = b & 0x1fu; return true; }
    switch (b) {
      case detail::kStr8: {
        uint8_t v;
        if (!get_byte_(v)) { fail_(saved); return false; }
        n = v;
        return true;
      }
      case detail::kStr16: {
        uint16_t v;
        if (!get_u16_(v)) { fail_(saved); return false; }
        n = v;
        return true;
      }
      case detail::kStr32: {
        uint32_t v;
        if (!get_u32_(v)) { fail_(saved); return false; }
        if (static_cast<uint64_t>(v) > detail::size_t_max_as_u64()) { fail_(saved); return false; }
        n = static_cast<size_t>(v);
        return true;
      }
      default:
        fail_(saved);
        return false;
    }
  }

  bool read_bin_header_(size_t &n) {
    const size_t saved = _pos;
    uint8_t b;
    if (!get_byte_(b)) { fail_(saved); return false; }
    switch (b) {
      case detail::kBin8: {
        uint8_t v;
        if (!get_byte_(v)) { fail_(saved); return false; }
        n = v;
        return true;
      }
      case detail::kBin16: {
        uint16_t v;
        if (!get_u16_(v)) { fail_(saved); return false; }
        n = v;
        return true;
      }
      case detail::kBin32: {
        uint32_t v;
        if (!get_u32_(v)) { fail_(saved); return false; }
        if (static_cast<uint64_t>(v) > detail::size_t_max_as_u64()) { fail_(saved); return false; }
        n = static_cast<size_t>(v);
        return true;
      }
      default:
        fail_(saved);
        return false;
    }
  }

  const uint8_t *_buf;
  size_t _size;
  size_t _pos;
  bool _error;
};

} // namespace msgpack
} // namespace serial_rpc
