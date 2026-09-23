// Tests for arduino/SerialRPC/src/serial_rpc/msgpack_lite.h
//
// Covers: round-trips for every object type and every integer-size
// boundary (fixint/8/16/32/64, both signs), str8/16/32 and bin8/16/32
// length boundaries, array/map header boundaries, float/double,
// numeric-coercion rules, Writer overflow, Reader truncation, skip() over
// nested structures, and a handful of literal byte vectors from the
// MessagePack spec.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "serial_rpc/msgpack_lite.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

using serial_rpc::msgpack::Reader;
using serial_rpc::msgpack::Writer;

namespace {

/// Packs `value` alone into a fresh buffer and reads it back into a
/// `ReadT`, asserting no Writer overflow. Returns the Reader so the caller
/// can make further assertions (error(), exact value, etc).
template <typename ReadT, typename PackT>
Reader pack_and_read(std::vector<uint8_t> &storage, PackT value, ReadT &out) {
  storage.assign(32, 0);
  Writer w(storage.data(), storage.size());
  w.pack(value);
  REQUIRE_FALSE(w.overflow());
  Reader r(storage.data(), w.size());
  r.read(out);
  return r;
}

template <typename T, typename PackT>
void expect_round_trip(PackT value) {
  std::vector<uint8_t> storage;
  T out{};
  Reader r = pack_and_read(storage, value, out);
  CHECK_FALSE(r.error());
  CHECK(out == static_cast<T>(value));
}

template <typename T, typename PackT>
void expect_coercion_error(PackT value, T sentinel) {
  std::vector<uint8_t> storage;
  T out = sentinel;
  Reader r = pack_and_read(storage, value, out);
  CHECK(r.error());
  CHECK(out == sentinel); // untouched on failure
}

std::vector<uint8_t> bytes_of(const Writer &w) {
  return std::vector<uint8_t>(w.data(), w.data() + w.size());
}

} // namespace

TEST_CASE("nil round-trip") {
  uint8_t buf[8];
  Writer w(buf, sizeof(buf));
  w.pack_nil();
  CHECK_FALSE(w.overflow());
  CHECK(bytes_of(w) == std::vector<uint8_t>{0xc0});

  Reader r(buf, w.size());
  CHECK(r.type() == Reader::Type::nil);
  r.read_nil();
  CHECK_FALSE(r.error());
  CHECK(r.remaining() == 0);
}

TEST_CASE("bool round-trip") {
  uint8_t buf[8];
  Writer w(buf, sizeof(buf));
  w.pack(true);
  w.pack(false);
  CHECK(bytes_of(w) == (std::vector<uint8_t>{0xc3, 0xc2}));

  Reader r(buf, w.size());
  CHECK(r.type() == Reader::Type::boolean);
  bool a = false, b = true;
  r.read(a);
  r.read(b);
  CHECK_FALSE(r.error());
  CHECK(a == true);
  CHECK(b == false);
}

TEST_CASE("positive fixint boundaries") {
  expect_round_trip<int>(0);
  expect_round_trip<int>(127);
  expect_round_trip<unsigned char>(0);
  expect_round_trip<unsigned char>(127);
}

TEST_CASE("uint8 boundary (128, 255)") {
  expect_round_trip<int>(128);
  expect_round_trip<int>(255);
  expect_round_trip<unsigned char>(255);
}

TEST_CASE("uint16 boundary (256, 65535)") {
  expect_round_trip<int>(256);
  expect_round_trip<int>(65535);
  expect_round_trip<unsigned short>(65535);
}

TEST_CASE("uint32 boundary (65536, 4294967295)") {
  expect_round_trip<long long>(65536LL);
  expect_round_trip<unsigned long long>(4294967295ULL);
}

TEST_CASE("uint64 boundary (4294967296, UINT64_MAX)") {
  expect_round_trip<unsigned long long>(4294967296ULL);
  expect_round_trip<unsigned long long>(0xffffffffffffffffULL);
}

TEST_CASE("negative fixint boundary (-1, -32)") {
  expect_round_trip<int>(-1);
  expect_round_trip<int>(-32);
  expect_round_trip<signed char>(-1);
  expect_round_trip<signed char>(-32);
}

TEST_CASE("int8 boundary (-33, -128)") {
  expect_round_trip<int>(-33);
  expect_round_trip<int>(-128);
  expect_round_trip<signed char>(-128);
}

TEST_CASE("int16 boundary (-129, -32768)") {
  expect_round_trip<int>(-129);
  expect_round_trip<int>(-32768);
  expect_round_trip<short>(-32768);
}

TEST_CASE("int32 boundary (-32769, -2147483648)") {
  expect_round_trip<long long>(-32769LL);
  expect_round_trip<long long>(-2147483648LL);
  expect_round_trip<int>(-2147483648LL);
}

TEST_CASE("int64 boundary (-2147483649, INT64_MIN)") {
  expect_round_trip<long long>(-2147483649LL);
  expect_round_trip<long long>(-9223372036854775807LL - 1); // INT64_MIN
}

TEST_CASE("every concrete pack()/read() overload round-trips") {
  expect_round_trip<signed char>(static_cast<signed char>(-5));
  expect_round_trip<short>(static_cast<short>(-5000));
  expect_round_trip<int>(-500000);
  expect_round_trip<long>(-500000L);
  expect_round_trip<long long>(-500000LL);
  expect_round_trip<unsigned char>(static_cast<unsigned char>(5));
  expect_round_trip<unsigned short>(static_cast<unsigned short>(50000));
  expect_round_trip<unsigned int>(500000U);
  expect_round_trip<unsigned long>(500000UL);
  expect_round_trip<unsigned long long>(500000ULL);
}

TEST_CASE("stdint typedefs resolve unambiguously") {
  expect_round_trip<int8_t>(static_cast<int8_t>(-100));
  expect_round_trip<uint8_t>(static_cast<uint8_t>(200));
  expect_round_trip<int16_t>(static_cast<int16_t>(-30000));
  expect_round_trip<uint16_t>(static_cast<uint16_t>(60000));
  expect_round_trip<int32_t>(static_cast<int32_t>(-2000000000));
  expect_round_trip<uint32_t>(static_cast<uint32_t>(4000000000U));
  expect_round_trip<int64_t>(static_cast<int64_t>(-9000000000000000000LL));
  expect_round_trip<uint64_t>(static_cast<uint64_t>(18000000000000000000ULL));
}

TEST_CASE("float round-trip") {
  uint8_t buf[8];
  Writer w(buf, sizeof(buf));
  w.pack(3.5f);
  CHECK(w.size() == 5);
  CHECK(buf[0] == 0xca);

  Reader r(buf, w.size());
  CHECK(r.type() == Reader::Type::float_);
  float out = 0.0f;
  r.read(out);
  CHECK_FALSE(r.error());
  CHECK(out == doctest::Approx(3.5f));
}

TEST_CASE("double round-trip (host: 8-byte double)") {
  uint8_t buf[16];
  Writer w(buf, sizeof(buf));
  w.pack(2.71828182845904523536);
  bool narrow_double = (sizeof(double) == sizeof(float));
  if (!narrow_double) {
    CHECK(w.size() == 9);
    CHECK(buf[0] == 0xcb);
  }

  Reader r(buf, w.size());
  CHECK(r.type() == Reader::Type::float_);
  double out = 0.0;
  r.read(out);
  CHECK_FALSE(r.error());
  CHECK(out == doctest::Approx(2.71828182845904523536));
}

TEST_CASE("numeric coercion: uint 200 -> uint8_t OK, int8_t errors") {
  expect_round_trip<uint8_t>(static_cast<unsigned>(200));
  expect_coercion_error<int8_t>(static_cast<unsigned>(200), static_cast<int8_t>(-7));
}

TEST_CASE("numeric coercion: -1 -> unsigned errors") {
  expect_coercion_error<uint8_t>(-1, static_cast<uint8_t>(42));
  expect_coercion_error<uint32_t>(-1, static_cast<uint32_t>(42));
}

TEST_CASE("numeric coercion: int -> float/double OK") {
  expect_round_trip<float>(42);
  expect_round_trip<double>(-1234);

  // Negative-magnitude and large-uint paths of read(float&)/read(double&).
  {
    std::vector<uint8_t> storage;
    float f = 0.0f;
    Reader r = pack_and_read(storage, -7, f);
    CHECK_FALSE(r.error());
    CHECK(f == doctest::Approx(-7.0f));
  }
  {
    std::vector<uint8_t> storage;
    double d = 0.0;
    Reader r = pack_and_read(storage, 0xffffffffffffffffULL, d);
    CHECK_FALSE(r.error());
    CHECK(d == doctest::Approx(18446744073709551615.0));
  }
}

TEST_CASE("str boundary: fixstr/str8 at 31 vs 32 bytes") {
  const std::string s31(31, 'x');
  const std::string s32(32, 'x');

  uint8_t buf[64];
  Writer w1(buf, sizeof(buf));
  w1.pack_str(s31.data(), s31.size());
  CHECK(w1.data()[0] == static_cast<uint8_t>(0xa0 | 31));
  CHECK(w1.size() == 1 + 31);

  Writer w2(buf, sizeof(buf));
  w2.pack_str(s32.data(), s32.size());
  CHECK(w2.data()[0] == 0xd9); // str8
  CHECK(w2.size() == 2 + 32);

  Reader r(w2.data(), w2.size());
  const char *ptr = nullptr;
  size_t len = 0;
  r.read_str(ptr, len);
  CHECK_FALSE(r.error());
  REQUIRE(len == 32);
  CHECK(std::string(ptr, len) == s32);
}

TEST_CASE("str boundary: str8/str16 at 255 vs 256 bytes") {
  const std::string s255(255, 'y');
  const std::string s256(256, 'y');
  std::vector<uint8_t> buf(300);

  Writer w1(buf.data(), buf.size());
  w1.pack_str(s255.data(), s255.size());
  CHECK(w1.data()[0] == 0xd9);
  CHECK(w1.size() == 2 + 255);

  Writer w2(buf.data(), buf.size());
  w2.pack_str(s256.data(), s256.size());
  CHECK(w2.data()[0] == 0xda); // str16
  CHECK(w2.size() == 3 + 256);

  char dst[300];
  Reader r(w2.data(), w2.size());
  r.read_str(dst, sizeof(dst));
  CHECK_FALSE(r.error());
  CHECK(std::string(dst) == s256);
}

TEST_CASE("str boundary: str16/str32 at 65535 vs 65536 bytes") {
  const std::string s16max(65535, 'z');
  const std::string s32min(65536, 'z');
  std::vector<uint8_t> buf(70000);

  Writer w1(buf.data(), buf.size());
  w1.pack_str(s16max.data(), s16max.size());
  CHECK(w1.data()[0] == 0xda);
  CHECK(w1.size() == 3 + 65535);

  Writer w2(buf.data(), buf.size());
  w2.pack_str(s32min.data(), s32min.size());
  CHECK(w2.data()[0] == 0xdb); // str32
  CHECK(w2.size() == 5 + 65536);

  Reader r(w2.data(), w2.size());
  const char *ptr = nullptr;
  size_t len = 0;
  r.read_str(ptr, len);
  CHECK_FALSE(r.error());
  CHECK(len == 65536);
}

TEST_CASE("read_str(char*,cap) fails cleanly when destination is too small") {
  uint8_t buf[16];
  Writer w(buf, sizeof(buf));
  w.pack_str("hello", 5);

  char dst[4] = {'?', '?', '?', '?'}; // room for only 3 chars + NUL
  Reader r(buf, w.size());
  r.read_str(dst, sizeof(dst));
  CHECK(r.error());
  CHECK(dst[0] == '?'); // untouched on failure
}

TEST_CASE("bin boundaries: bin8/bin16/bin32") {
  std::vector<uint8_t> payload255(255, 0xab);
  std::vector<uint8_t> payload256(256, 0xcd);
  std::vector<uint8_t> buf(70000);

  Writer w1(buf.data(), buf.size());
  w1.pack_bin(payload255.data(), payload255.size());
  CHECK(w1.data()[0] == 0xc4); // bin8
  CHECK(w1.size() == 2 + 255);

  Writer w2(buf.data(), buf.size());
  w2.pack_bin(payload256.data(), payload256.size());
  CHECK(w2.data()[0] == 0xc5); // bin16
  CHECK(w2.size() == 3 + 256);

  Reader r(w2.data(), w2.size());
  CHECK(r.type() == Reader::Type::bin);
  const uint8_t *ptr = nullptr;
  size_t len = 0;
  r.read_bin(ptr, len);
  CHECK_FALSE(r.error());
  REQUIRE(len == 256);
  CHECK(std::memcmp(ptr, payload256.data(), len) == 0);

  std::vector<uint8_t> payload65536(65536, 0xef);
  Writer w3(buf.data(), buf.size());
  w3.pack_bin(payload65536.data(), payload65536.size());
  CHECK(w3.data()[0] == 0xc6); // bin32
  CHECK(w3.size() == 5 + 65536);
}

TEST_CASE("array header boundary: fixarray/array16 at 15 vs 16 elements") {
  uint8_t buf[8];
  Writer w1(buf, sizeof(buf));
  w1.pack_array(15);
  CHECK(w1.data()[0] == static_cast<uint8_t>(0x90 | 15));
  CHECK(w1.size() == 1);

  Writer w2(buf, sizeof(buf));
  w2.pack_array(16);
  CHECK(w2.data()[0] == 0xdc); // array16
  CHECK(w2.size() == 3);

  size_t n = 0;
  Reader r(w2.data(), w2.size());
  CHECK(r.type() == Reader::Type::array);
  r.read_array(n);
  CHECK_FALSE(r.error());
  CHECK(n == 16);
}

TEST_CASE("map header boundary: fixmap/map16 at 15 vs 16 pairs") {
  uint8_t buf[8];
  Writer w1(buf, sizeof(buf));
  w1.pack_map(15);
  CHECK(w1.data()[0] == static_cast<uint8_t>(0x80 | 15));

  Writer w2(buf, sizeof(buf));
  w2.pack_map(16);
  CHECK(w2.data()[0] == 0xde); // map16
  CHECK(w2.size() == 3);

  size_t n = 0;
  Reader r(w2.data(), w2.size());
  CHECK(r.type() == Reader::Type::map);
  r.read_map(n);
  CHECK_FALSE(r.error());
  CHECK(n == 16);
}

TEST_CASE("pack_all packs a mixed argument list in order") {
  uint8_t buf[32];
  Writer w(buf, sizeof(buf));
  w.pack_all(13, true, "hi");

  Reader r(buf, w.size());
  int i = 0;
  bool b = false;
  const char *s = nullptr;
  size_t len = 0;
  r.read(i);
  r.read(b);
  r.read_str(s, len);
  CHECK_FALSE(r.error());
  CHECK(i == 13);
  CHECK(b == true);
  CHECK(std::string(s, len) == "hi");
}

TEST_CASE("Writer overflow is sticky and never writes past capacity") {
  uint8_t buf[2] = {0xff, 0xff};
  Writer w(buf, sizeof(buf));
  w.pack(12345); // needs 3 bytes (uint16 header + 2), buffer only holds 2
  CHECK(w.overflow());
  CHECK(w.size() <= sizeof(buf));

  // A subsequent, smaller write keeps overflow() sticky.
  w.pack_nil();
  CHECK(w.overflow());
}

TEST_CASE("Writer::reset clears size and overflow") {
  uint8_t buf[2];
  Writer w(buf, sizeof(buf));
  w.pack(999999);
  CHECK(w.overflow());
  w.reset();
  CHECK_FALSE(w.overflow());
  CHECK(w.size() == 0);
}

TEST_CASE("Reader detects truncated input and leaves output unchanged") {
  // uint16 marker claiming 2 more bytes follow, but the buffer ends right after it.
  uint8_t buf[1] = {0xcd};
  Reader r(buf, sizeof(buf));
  int out = 42;
  r.read(out);
  CHECK(r.error());
  CHECK(out == 42);
}

TEST_CASE("Reader detects a str header whose payload is truncated") {
  uint8_t buf[3] = {0xd9, 10, 'a'}; // str8, claims length 10, only 1 byte follows
  Reader r(buf, sizeof(buf));
  const char *ptr = nullptr;
  size_t len = 0;
  r.read_str(ptr, len);
  CHECK(r.error());
}

TEST_CASE("once error() is set, position stays put for further reads") {
  uint8_t buf[1] = {0xcd}; // truncated uint16
  Reader r(buf, sizeof(buf));
  int out = 0;
  r.read(out);
  REQUIRE(r.error());
  const size_t pos_after_first_failure = r.remaining();
  bool b = false;
  r.read(b);
  CHECK(r.error());
  CHECK(r.remaining() == pos_after_first_failure);
}

TEST_CASE("skip over a nested array/map structure lands exactly on the next sibling") {
  uint8_t buf[64];
  Writer w(buf, sizeof(buf));
  // [1, {"a": [2, 3]}, "x"]  followed by a sentinel nil.
  w.pack_array(3);
  w.pack(1);
  w.pack_map(1);
  w.pack_str("a", 1);
  w.pack_array(2);
  w.pack(2);
  w.pack(3);
  w.pack_str("x", 1);
  w.pack_nil(); // sentinel

  Reader r(buf, w.size());
  r.skip(); // should consume the whole 3-element array
  CHECK_FALSE(r.error());
  CHECK(r.type() == Reader::Type::nil);
  r.read_nil();
  CHECK_FALSE(r.error());
  CHECK(r.remaining() == 0);
}

TEST_CASE("skip over an ext type") {
  uint8_t buf[8] = {0xd4, 0x01, 0xab, 0xc0}; // fixext1(type=1,payload=1 byte), then a sentinel nil
  Reader r(buf, sizeof(buf));
  r.skip();
  CHECK_FALSE(r.error());
  CHECK(r.type() == Reader::Type::nil);
}

TEST_CASE("known byte vectors from the MessagePack spec") {
  uint8_t buf[16];

  {
    Writer w(buf, sizeof(buf));
    w.pack_nil();
    CHECK(bytes_of(w) == std::vector<uint8_t>{0xc0});
  }
  {
    Writer w(buf, sizeof(buf));
    w.pack(true);
    CHECK(bytes_of(w) == std::vector<uint8_t>{0xc3});
  }
  {
    Writer w(buf, sizeof(buf));
    w.pack(false);
    CHECK(bytes_of(w) == std::vector<uint8_t>{0xc2});
  }
  {
    Writer w(buf, sizeof(buf));
    w.pack(0);
    CHECK(bytes_of(w) == std::vector<uint8_t>{0x00});
  }
  {
    Writer w(buf, sizeof(buf));
    w.pack(127);
    CHECK(bytes_of(w) == std::vector<uint8_t>{0x7f});
  }
  {
    Writer w(buf, sizeof(buf));
    w.pack(-1);
    CHECK(bytes_of(w) == std::vector<uint8_t>{0xff});
  }
  {
    Writer w(buf, sizeof(buf));
    w.pack(128);
    CHECK(bytes_of(w) == (std::vector<uint8_t>{0xcc, 0x80}));
  }
  {
    Writer w(buf, sizeof(buf));
    w.pack_str("hello", 5);
    CHECK(bytes_of(w) == (std::vector<uint8_t>{0xa5, 'h', 'e', 'l', 'l', 'o'}));
  }
  {
    Writer w(buf, sizeof(buf));
    w.pack_array(3);
    w.pack(1);
    w.pack(2);
    w.pack(3);
    CHECK(bytes_of(w) == (std::vector<uint8_t>{0x93, 0x01, 0x02, 0x03}));
  }
}
