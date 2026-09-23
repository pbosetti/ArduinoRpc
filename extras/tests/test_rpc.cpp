// Tests for host/include/serial_rpc.hpp (docs/PLAN.md, component 5).
//
// Compiled twice (see tests/CMakeLists.txt), once against each device-side
// storage backend from server.h (docs/PLAN.md 3b): SERIAL_RPC_USE_STL=0
// (target serial_rpc_tests_host_fixed) and =1 (serial_rpc_tests_host_stl).
// Every handler bound on the device side below only uses argument/return
// types supported by *both* backends (bool, the fundamental integer types,
// float/double, const char*), so the exact same test file and bindings
// exercise the full host<->device round trip under either backend.
//
// Covers (docs/PLAN.md's implementation-order item 5): Value encode/decode
// round trips, as<T>() conversions and errors, to_string(), max-depth
// protection, parse_rpc_list() for both rpc.list shapes, and
// collect_rpc_list_pages() for a paging device, a device that ignores
// `start` (rpc.list's paging is a moving target on the device side as of
// this writing -- see the comment on RPC::list()), an empty list, and the
// max-pages guard; then, against a
// LoopbackPort bridging RPC<LoopbackPort> to a real in-process
// serial_rpc::Server<MockStream>: typed calls, RemoteError (unknown method,
// bad args), TimeoutError (a port that never answers), notifications both
// ways, logs detached (as text) vs. attached (via on_log), text routing,
// connect()/disconnect(), list(), text interleaved inside a pending call,
// and a stray response msgid being counted and ignored.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "serial_rpc.hpp"
#include "serial_rpc/server.h"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <thread>
#include <vector>

using serial_rpc::MethodInfo;
using serial_rpc::ProtocolError;
using serial_rpc::RemoteError;
using serial_rpc::TimeoutError;
using serial_rpc::TypeError;
using serial_rpc::Value;

// ===========================================================================
// Value: encode/decode, as<T>(), to_string(), depth limit
// ===========================================================================

TEST_CASE("Value: nil/bool/int/uint/double/string/bin round-trip through encode()/decode()") {
  auto round_trip = [](const Value &v) {
    uint8_t buf[256];
    serial_rpc::msgpack::Writer w(buf, sizeof(buf));
    v.encode(w);
    REQUIRE_FALSE(w.overflow());
    serial_rpc::msgpack::Reader r(buf, w.size());
    return Value::decode(r);
  };

  CHECK(round_trip(Value()).is_nil());
  CHECK(round_trip(Value(nullptr)).is_nil());
  CHECK(round_trip(Value(true)).as<bool>() == true);
  CHECK(round_trip(Value(false)).as<bool>() == false);
  CHECK(round_trip(Value(int64_t{-12345})).as<int64_t>() == -12345);
  CHECK(round_trip(Value(uint64_t{0xffffffffffull})).as<uint64_t>() == 0xffffffffffull);
  CHECK(round_trip(Value(3.5)).as<double>() == doctest::Approx(3.5));
  CHECK(round_trip(Value(std::string("hello"))).as<std::string>() == "hello");
  CHECK(round_trip(Value("literal")).as<std::string>() == "literal");
  const Value::Bin bin{1, 2, 3, 4, 5};
  CHECK(round_trip(Value(bin)).as<Value::Bin>() == bin);
}

TEST_CASE("Value: array and map round-trip, preserving element/key order") {
  Value::Array arr;
  arr.emplace_back(int64_t{1});
  arr.emplace_back(std::string("two"));
  arr.emplace_back(true);
  Value varr(arr);

  uint8_t buf[256];
  serial_rpc::msgpack::Writer w(buf, sizeof(buf));
  varr.encode(w);
  REQUIRE_FALSE(w.overflow());
  serial_rpc::msgpack::Reader r(buf, w.size());
  const Value decoded = Value::decode(r);
  REQUIRE(decoded.is_array());
  const auto out = decoded.as<std::vector<Value>>();
  REQUIRE(out.size() == 3);
  CHECK(out[0].as<int64_t>() == 1);
  CHECK(out[1].as<std::string>() == "two");
  CHECK(out[2].as<bool>() == true);

  Value::Map m;
  m.emplace_back(Value("z"), Value(int64_t{1}));
  m.emplace_back(Value("a"), Value(int64_t{2}));
  Value vmap(m);
  serial_rpc::msgpack::Writer w2(buf, sizeof(buf));
  vmap.encode(w2);
  REQUIRE_FALSE(w2.overflow());
  serial_rpc::msgpack::Reader r2(buf, w2.size());
  const Value decoded_map = Value::decode(r2);
  REQUIRE(decoded_map.is_map());
  // to_string() renders in stored (not sorted) order, so "z" before "a"
  // proves order survived the round trip.
  CHECK(decoded_map.to_string() == R"({"z": 1, "a": 2})");
}

TEST_CASE("Value: as<T> integer range checks throw TypeError on overflow or wrong wire type") {
  CHECK(Value(int64_t{200}).as<uint8_t>() == 200);
  CHECK_THROWS_AS(Value(int64_t{300}).as<uint8_t>(), TypeError);
  CHECK_THROWS_AS(Value(int64_t{-1}).as<uint8_t>(), TypeError);
  CHECK(Value(int64_t{-5}).as<int8_t>() == -5);
  CHECK_THROWS_AS(Value(uint64_t{300}).as<int8_t>(), TypeError);
  CHECK_THROWS_AS(Value(std::string("x")).as<int>(), TypeError);
  CHECK_THROWS_AS(Value(int64_t{1}).as<std::string>(), TypeError);
  CHECK_THROWS_AS(Value(int64_t{1}).as<bool>(), TypeError);
}

TEST_CASE("Value: as<T> floating/string/vector<T>/map<string,T> conversions") {
  CHECK(Value(int64_t{7}).as<double>() == doctest::Approx(7.0));
  CHECK(Value(uint64_t{7}).as<float>() == doctest::Approx(7.0f));
  CHECK(Value(2.5).as<float>() == doctest::Approx(2.5f));

  Value::Array arr{Value(int64_t{1}), Value(int64_t{2}), Value(int64_t{3})};
  const auto ints = Value(arr).as<std::vector<int>>();
  CHECK(ints == std::vector<int>{1, 2, 3});
  CHECK_THROWS_AS(Value(int64_t{1}).as<std::vector<int>>(), TypeError);

  using StrIntMap = std::map<std::string, int>; // a bare std::map<A,B> template argument
                                                 // would split CHECK_THROWS_AS's macro args on the comma
  Value::Map m;
  m.emplace_back(Value("a"), Value(int64_t{1}));
  m.emplace_back(Value("b"), Value(int64_t{2}));
  const auto sm = Value(m).as<StrIntMap>();
  REQUIRE(sm.size() == 2);
  CHECK(sm.at("a") == 1);
  CHECK(sm.at("b") == 2);
  CHECK_THROWS_AS(Value(int64_t{1}).as<StrIntMap>(), TypeError);
}

TEST_CASE("Value: to_string() produces compact, escaped, JSON-like output") {
  CHECK(Value().to_string() == "nil");
  CHECK(Value(true).to_string() == "true");
  CHECK(Value(int64_t{-3}).to_string() == "-3");
  CHECK(Value(std::string("a\"b\nc")).to_string() == R"("a\"b\nc")");
  CHECK(Value(Value::Bin{1, 2, 3}).to_string() == "<bin 3 bytes>");
  Value::Array arr{Value(int64_t{1}), Value(int64_t{2})};
  CHECK(Value(arr).to_string() == "[1, 2]");
}

TEST_CASE("Value: decode() throws ProtocolError past the max nesting depth") {
  uint8_t buf[256];
  serial_rpc::msgpack::Writer w(buf, sizeof(buf));
  const int depth = Value::kMaxDepth + 5;
  for (int i = 0; i < depth; ++i) w.pack_array(1);
  w.pack_nil();
  REQUIRE_FALSE(w.overflow());
  serial_rpc::msgpack::Reader r(buf, w.size());
  CHECK_THROWS_AS(Value::decode(r), ProtocolError);
}

TEST_CASE("parse_rpc_list: accepts both the plain-name and [name,signature] shapes") {
  Value::Array plain{Value("alpha"), Value("beta")};
  const auto mi_plain = serial_rpc::parse_rpc_list(Value(plain));
  REQUIRE(mi_plain.size() == 2);
  CHECK(mi_plain[0].name == "alpha");
  CHECK(mi_plain[0].signature == "");
  CHECK(mi_plain[1].name == "beta");

  Value::Array pairs;
  pairs.emplace_back(Value::Array{Value("set_led"), Value("(u8,bool)->nil")});
  pairs.emplace_back(Value::Array{Value("no_sig")}); // a 1-element array: name only
  const auto mi_pairs = serial_rpc::parse_rpc_list(Value(pairs));
  REQUIRE(mi_pairs.size() == 2);
  CHECK(mi_pairs[0].name == "set_led");
  CHECK(mi_pairs[0].signature == "(u8,bool)->nil");
  CHECK(mi_pairs[1].name == "no_sig");
  CHECK(mi_pairs[1].signature == "");

  CHECK_THROWS_AS(serial_rpc::parse_rpc_list(Value(int64_t{1})), ProtocolError);
}

TEST_CASE("collect_rpc_list_pages: pages through a device that honours 'start'") {
  const std::vector<MethodInfo> all = {{"a", ""}, {"b", ""}, {"c", ""}, {"d", ""}, {"e", ""}};
  int calls = 0;
  const auto pages = serial_rpc::collect_rpc_list_pages([&](int64_t start) {
    ++calls;
    std::vector<MethodInfo> page;
    for (int64_t i = start; i < static_cast<int64_t>(all.size()) && page.size() < 2; ++i) page.push_back(all[i]);
    return page;
  });
  REQUIRE(pages.size() == 5);
  for (size_t i = 0; i < all.size(); ++i) CHECK(pages[i].name == all[i].name);
  CHECK(calls == 4); // two 2-entry pages, a final 1-entry page, then the empty page
}

TEST_CASE("collect_rpc_list_pages: stays compatible with a device that ignores 'start'") {
  const std::vector<MethodInfo> all = {{"a", ""}, {"b", ""}, {"c", ""}};
  int calls = 0;
  const auto pages = serial_rpc::collect_rpc_list_pages([&](int64_t /*start*/) {
    ++calls;
    return all; // always the full set, regardless of start
  });
  REQUIRE(pages.size() == 3);
  for (size_t i = 0; i < all.size(); ++i) CHECK(pages[i].name == all[i].name);
  // First call returns everything; the second call (start = 3) returns the
  // same set again, which adds nothing new, so the loop stops there.
  CHECK(calls == 2);
}

TEST_CASE("collect_rpc_list_pages: an immediately-empty first page yields no methods") {
  int calls = 0;
  const auto pages = serial_rpc::collect_rpc_list_pages([&](int64_t) {
    ++calls;
    return std::vector<MethodInfo>{};
  });
  CHECK(pages.empty());
  CHECK(calls == 1);
}

TEST_CASE("collect_rpc_list_pages: a device that never signals the end is stopped by the max-pages guard") {
  int calls = 0;
  const auto pages = serial_rpc::collect_rpc_list_pages([&](int64_t start) {
    ++calls;
    // Always exactly one brand-new, never-before-seen name: every page
    // "adds something new" forever, so only the iteration guard can stop this.
    return std::vector<MethodInfo>{{"m" + std::to_string(start), ""}};
  });
  CHECK(calls == serial_rpc::kMaxListPages);
  CHECK(pages.size() == static_cast<size_t>(serial_rpc::kMaxListPages));
}

// ===========================================================================
// LoopbackPort: bridges RPC<LoopbackPort> to a real, in-process
// serial_rpc::Server<MockStream>, single-threaded. write() queues bytes for
// the device; read_some() lets the device consume them (server.poll()) and
// then hands back whatever the device produced, honouring the port's own
// timeout by actually waiting when nothing is available -- this is what
// lets RPC's real deadline-driven retry loops (call(), connect(), poll())
// be exercised faithfully, including the "port that never answers" case.
// ===========================================================================

namespace {

struct MockStream {
  std::vector<uint8_t> rx;
  size_t rx_pos = 0;
  std::vector<uint8_t> tx;
  size_t tx_pos = 0;

  int available() { return static_cast<int>(rx.size() - rx_pos); }
  int read() { return rx_pos < rx.size() ? rx[rx_pos++] : -1; }
  size_t write(const uint8_t *buf, size_t n) {
    tx.insert(tx.end(), buf, buf + n);
    return n;
  }
  size_t write(uint8_t b) {
    tx.push_back(b);
    return 1;
  }
};

using TestServer = serial_rpc::Server<MockStream, 8, 128>;

class LoopbackPort {
public:
  LoopbackPort() : _server(_stream) { _server.begin(); }

  TestServer &server() { return _server; }

  std::chrono::milliseconds timeout() const { return _timeout; }
  void set_timeout(std::chrono::milliseconds t) { _timeout = t; }

  int write(const char *buf, size_t n) {
    if (!_drop_writes) _stream.rx.insert(_stream.rx.end(), buf, buf + n);
    return static_cast<int>(n);
  }

  int read_some(char *buf, size_t n_max) {
    _server.poll();
    if (_stream.tx_pos < _stream.tx.size()) {
      const size_t avail = _stream.tx.size() - _stream.tx_pos;
      const size_t n = std::min(avail, n_max);
      std::memcpy(buf, _stream.tx.data() + _stream.tx_pos, n);
      _stream.tx_pos += n;
      return static_cast<int>(n);
    }
    if (_timeout.count() > 0) std::this_thread::sleep_for(_timeout);
    return 0;
  }

  void set_dtr(bool on) { _dtr = on; }
  bool dtr() const { return _dtr; }

  void flush_input() {
    _stream.tx.erase(_stream.tx.begin(), _stream.tx.begin() + static_cast<std::ptrdiff_t>(_stream.tx_pos));
    _stream.tx_pos = 0;
  }

  /// Test-only: once set, write() drops bytes on the floor instead of
  /// delivering them, modelling "a port that never answers".
  void drop_writes(bool drop) { _drop_writes = drop; }

  /// Test-only: appends a raw text line directly to the device's outgoing
  /// stream, as if the device had printed it, without going through
  /// Server -- used to put text in front of / inside a pending call's
  /// response.
  void inject_device_text(std::string_view line) {
    _stream.tx.insert(_stream.tx.end(), line.begin(), line.end());
    _stream.tx.push_back('\n');
  }

  /// Test-only: appends an already-framed byte blob directly to the
  /// device's outgoing stream -- used to simulate a stray response with an
  /// unrelated msgid.
  void inject_frame(const uint8_t *payload, size_t len) {
    uint8_t framed[256];
    const size_t n = serial_rpc::frame_encode(payload, len, framed, sizeof(framed));
    REQUIRE(n != 0);
    _stream.tx.insert(_stream.tx.end(), framed, framed + n);
  }

private:
  MockStream _stream;
  TestServer _server;
  std::chrono::milliseconds _timeout{1000};
  bool _dtr = false;
  bool _drop_writes = false;
};

using TestRpc = serial_rpc::RPC<LoopbackPort>;

} // namespace

// ===========================================================================
// LoopbackPort-based RPC<LoopbackPort> <-> Server<MockStream> integration
// ===========================================================================

TEST_CASE("RPC: connect() attaches, and ping() returns the protocol version") {
  LoopbackPort port;
  TestRpc rpc(port);
  CHECK_FALSE(rpc.attached());
  rpc.connect();
  CHECK(rpc.attached());
  CHECK(port.server().attached());
  CHECK(rpc.ping() == static_cast<int>(serial_rpc::kProtocolVersion));
}

TEST_CASE("RPC: typed call<int> round trip") {
  LoopbackPort port;
  port.server().bind("add", [](int a, int b) { return a + b; });
  TestRpc rpc(port);

  CHECK(rpc.call<int>("add", 2, 3) == 5);
  CHECK(rpc.last_rtt() >= std::chrono::milliseconds(0));
  CHECK(rpc.stats().calls == 1);
}

TEST_CASE("RPC: call() dynamic Value round trip") {
  LoopbackPort port;
  port.server().bind("mul", [](int a, int b) { return a * b; });
  TestRpc rpc(port);

  const Value v = rpc.call("mul", 6, 7);
  CHECK(v.as<int>() == 42);
}

TEST_CASE("RPC: unknown method throws RemoteError") {
  LoopbackPort port;
  TestRpc rpc(port);

  bool threw = false;
  try {
    rpc.call("does_not_exist");
  } catch (const RemoteError &e) {
    threw = true;
    CHECK(e.method() == "does_not_exist");
    CHECK(e.error_value().is_string());
    CHECK(e.error_value().as<std::string>() == "unknown method");
  }
  CHECK(threw);
}

TEST_CASE("RPC: bad argument count or type throws RemoteError") {
  LoopbackPort port;
  port.server().bind("add", [](int a, int b) { return a + b; });
  TestRpc rpc(port);

  CHECK_THROWS_AS(rpc.call("add", 1), RemoteError); // missing an argument
  CHECK_THROWS_AS(rpc.call("add", "nope", 2), RemoteError); // wrong argument type
}

TEST_CASE("RPC: a port that never answers throws TimeoutError") {
  LoopbackPort port;
  port.drop_writes(true);
  TestRpc::Options opts;
  opts.timeout = std::chrono::milliseconds(200);
  TestRpc rpc(port, opts);

  CHECK_THROWS_AS(rpc.call("rpc.ping"), TimeoutError);
  CHECK(rpc.stats().timeouts == 1);
}

TEST_CASE("RPC: notify() host->device reaches the bound handler with no reply") {
  LoopbackPort port;
  static int g_pin = -1;
  static bool g_on = false;
  port.server().bind("set_led", [](int pin, bool on) {
    g_pin = pin;
    g_on = on;
  });
  TestRpc rpc(port);

  rpc.notify("set_led", 13, true);
  rpc.poll(std::chrono::milliseconds(20)); // give the loopback a pump cycle
  CHECK(g_pin == 13);
  CHECK(g_on == true);
  CHECK(rpc.stats().notifications_sent == 1);
}

TEST_CASE("RPC: a device notification reaches on() and on_any_notification") {
  LoopbackPort port;
  TestRpc rpc(port);
  rpc.connect();

  std::vector<int> specific_seen;
  std::vector<std::string> any_seen;
  rpc.on("evt", [&](const Value &args) { specific_seen.push_back(args.as<std::vector<Value>>().at(0).as<int>()); });
  rpc.on_any_notification([&](std::string_view method, const Value &) { any_seen.emplace_back(method); });

  CHECK(port.server().notify("evt", 42));
  const size_t n = rpc.poll(std::chrono::milliseconds(50));
  CHECK(n >= 1);
  REQUIRE(specific_seen.size() == 1);
  CHECK(specific_seen[0] == 42);
  REQUIRE(any_seen.size() == 1);
  CHECK(any_seen[0] == "evt");
  CHECK(rpc.stats().notifications_recv == 1);
}

TEST_CASE("RPC: rpc.log routes as a text line while the device is detached") {
  LoopbackPort port;
  TestRpc rpc(port);
  CHECK_FALSE(rpc.attached());

  std::vector<std::string> lines;
  rpc.on_text([&](std::string_view l) { lines.emplace_back(l); });
  port.server().log.println("hello, detached world");

  const size_t n = rpc.poll(std::chrono::milliseconds(50));
  CHECK(n >= 1);
  bool found = false;
  for (const auto &l : lines) {
    if (l == "hello, detached world") found = true;
  }
  CHECK(found);
  CHECK(rpc.stats().logs_recv == 0); // it went out as plain text, not a framed "log" notification
}

TEST_CASE("RPC: rpc.log routes via on_log while the device is attached") {
  LoopbackPort port;
  TestRpc rpc(port);
  rpc.connect();

  std::vector<std::pair<int, std::string>> logs;
  rpc.on_log([&](int level, std::string_view line) { logs.emplace_back(level, std::string(line)); });
  port.server().log_at(3).println("hello, attached world");

  const size_t n = rpc.poll(std::chrono::milliseconds(50));
  CHECK(n >= 1);
  REQUIRE(logs.size() == 1);
  CHECK(logs[0].first == 3);
  CHECK(logs[0].second == "hello, attached world");
  CHECK(rpc.stats().logs_recv == 1);
}

TEST_CASE("RPC: send_text() reaches the device's on_text callback") {
  LoopbackPort port;
  static std::string g_seen;
  port.server().on_text([](const char *line) { g_seen = line; });
  TestRpc rpc(port);

  rpc.send_text("hello from host");
  port.server().poll(); // deliver it on the device side
  CHECK(g_seen == "hello from host");
}

TEST_CASE("RPC: list() normalizes a plain-name rpc.list result") {
  LoopbackPort port;
  port.server().bind("alpha", []() {});
  port.server().bind("beta", []() {});
  TestRpc rpc(port);

  const std::vector<MethodInfo> methods = rpc.list();
  REQUIRE(methods.size() == 2);
  CHECK(methods[0].name == "alpha");
  CHECK(methods[1].name == "beta");
}

TEST_CASE("RPC: connect()/disconnect() attach and detach the device") {
  LoopbackPort port;
  TestRpc rpc(port);
  rpc.connect();
  CHECK(rpc.attached());
  CHECK(port.server().attached());

  rpc.disconnect();
  CHECK_FALSE(rpc.attached());
  // Give the loopback a pump cycle to deliver the rpc.detach notification.
  port.server().poll();
  CHECK_FALSE(port.server().attached());
}

TEST_CASE("RPC: connect(reset=true) pulses DTR and still attaches") {
  LoopbackPort port;
  TestRpc rpc(port);
  rpc.connect(std::chrono::milliseconds(3000), /*reset=*/true);
  CHECK(rpc.attached());
  CHECK(port.dtr()); // left high after the pulse
}

TEST_CASE("RPC: text interleaved before the response is still delivered during a pending call") {
  LoopbackPort port;
  port.server().bind("slow", []() { return 1; });
  TestRpc rpc(port);

  std::vector<std::string> lines;
  rpc.on_text([&](std::string_view l) { lines.emplace_back(l); });
  // Sits in the device's outgoing stream ahead of the eventual response,
  // simulating a Serial.print() the sketch made moments earlier.
  port.inject_device_text("interleaved diagnostic line");

  const int result = rpc.call<int>("slow");
  CHECK(result == 1);
  bool found = false;
  for (const auto &l : lines) {
    if (l == "interleaved diagnostic line") found = true;
  }
  CHECK(found);
}

TEST_CASE("RPC: a stray response msgid is counted and ignored, the real call still completes") {
  LoopbackPort port;
  port.server().bind("id", [](int x) { return x; });
  TestRpc rpc(port);

  // A response for a msgid nobody is waiting on: [1, 999999, nil, 123].
  uint8_t payload[32];
  serial_rpc::msgpack::Writer w(payload, sizeof(payload));
  w.pack_array(4);
  w.pack(1);
  w.pack(999999);
  w.pack_nil();
  w.pack(123);
  REQUIRE_FALSE(w.overflow());
  port.inject_frame(payload, w.size());

  CHECK(rpc.call<int>("id", 7) == 7);
  CHECK(rpc.stats().stray_responses == 1);
}
