// Tests for arduino/SerialRPC/src/serial_rpc/server.h
//
// Compiled twice (see tests/CMakeLists.txt): once with -DSERIAL_RPC_USE_STL=0
// (target serial_rpc_tests_fixed) and once with -DSERIAL_RPC_USE_STL=1
// (target serial_rpc_tests_stl), so both storage backends run every shared
// test below; a few STL-only cases (std::string/vector/array/pair/tuple
// arguments) are guarded by `#if SERIAL_RPC_USE_STL` and only run in the
// second target.
//
// Covers (see docs/PLAN.md's implementation-order item 3): bind/dispatch for
// free functions, capturing lambdas and member functions; argument-count and
// type errors; unknown methods; raw handlers; rpc.list; rpc.ping; attach,
// detach and the attach timeout; rpc.log routed as text when detached and as
// frames when attached; on_text; notifications only while attached; the
// map-style bind proxy; const char* string arguments; and, in the STL
// target, the extra STL argument/return types.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "serial_rpc/server.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#if SERIAL_RPC_USE_STL
#include <array>
#include <tuple>
#include <utility>
#endif

using serial_rpc::Args;
using serial_rpc::Demux;
using serial_rpc::DemuxEvent;
using serial_rpc::Reply;
using serial_rpc::frame_encode;
using serial_rpc::msgpack::Reader;
using serial_rpc::msgpack::Writer;

namespace {

// ---------------------------------------------------------------------------
// A loopback mock Stream: rx is filled by the test before poll(), tx is
// captured for the test to decode afterwards. Duck-typed exactly like
// Arduino's Stream/Print (available/read/write), per server.h's StreamT
// requirements.
struct MockStream {
  std::vector<uint8_t> rx;
  size_t rx_pos = 0;
  std::vector<uint8_t> tx;

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

  /// Frames `payload` and appends it to `rx`, so a subsequent `poll()` sees it.
  void feed_frame(const uint8_t *payload, size_t len) {
    uint8_t framed[1024];
    const size_t n = frame_encode(payload, len, framed, sizeof(framed));
    REQUIRE(n != 0);
    rx.insert(rx.end(), framed, framed + n);
  }

  /// Appends raw (unframed) text bytes to `rx`.
  void feed_text(const char *line) {
    rx.insert(rx.end(), line, line + strlen(line));
  }
};

using TestServer = serial_rpc::Server<MockStream, 8, 128>;

template <class... A>
void send_request(MockStream &s, uint32_t msgid, const char *method, const A &...args) {
  uint8_t payload[256];
  Writer w(payload, sizeof(payload));
  w.pack_array(4);
  w.pack(0);
  w.pack(msgid);
  w.pack(method);
  w.pack_array(sizeof...(A));
  w.pack_all(args...);
  REQUIRE_FALSE(w.overflow());
  s.feed_frame(payload, w.size());
}

template <class... A>
void send_notification(MockStream &s, const char *method, const A &...args) {
  uint8_t payload[256];
  Writer w(payload, sizeof(payload));
  w.pack_array(3);
  w.pack(2);
  w.pack(method);
  w.pack_array(sizeof...(A));
  w.pack_all(args...);
  REQUIRE_FALSE(w.overflow());
  s.feed_frame(payload, w.size());
}

/// Every frame (payload bytes, COBS/CRC stripped) found in `tx`, in order.
std::vector<std::vector<uint8_t>> decode_frames(const std::vector<uint8_t> &tx) {
  std::vector<std::vector<uint8_t>> frames;
  Demux<1024, 128> demux;
  for (uint8_t b : tx) {
    if (demux.feed(b) == DemuxEvent::frame) {
      frames.emplace_back(demux.data(), demux.data() + demux.size());
    }
  }
  return frames;
}

/// Every plain text line found in `tx` (bytes not part of a frame).
std::vector<std::string> decode_text_lines(const std::vector<uint8_t> &tx) {
  std::vector<std::string> lines;
  Demux<1024, 128> demux;
  for (uint8_t b : tx) {
    if (demux.feed(b) == DemuxEvent::text_line) lines.emplace_back(demux.line());
  }
  return lines;
}

struct ParsedResponse {
  bool well_formed = false;
  uint32_t msgid = 0;
  bool is_error = false;
  std::string error_msg;
  size_t result_pos = 0; // offset into `payload` where the result value starts
};

/// Parses `[1, msgid, error|nil, result]`. The result itself is left
/// un-decoded (`result_pos` into the same `payload` buffer); callers build
/// their own Reader from there for type-specific assertions.
ParsedResponse parse_response(const std::vector<uint8_t> &payload) {
  ParsedResponse pr;
  Reader r(payload.data(), payload.size());
  size_t n;
  r.read_array(n);
  int type;
  r.read(type);
  if (r.error() || n != 4 || type != 1) return pr;
  r.read(pr.msgid);
  if (r.type() == Reader::Type::nil) {
    r.read_nil();
  } else {
    const char *p;
    size_t l;
    r.read_str(p, l);
    pr.is_error = true;
    pr.error_msg.assign(p, l);
  }
  if (r.error()) return pr;
  pr.result_pos = payload.size() - r.remaining();
  pr.well_formed = true;
  return pr;
}

/// Fake millis()-like clock, injectable via Server::set_clock(), so the
/// attach-timeout tests don't depend on wall-clock time.
uint32_t g_fake_now = 0;
uint32_t fake_clock() { return g_fake_now; }

} // namespace

// ===========================================================================
// bind() / dispatch: free functions, lambdas, member functions
// ===========================================================================

TEST_CASE("bind: free function, dispatched by request") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();
  s.tx.clear();

  static bool called = false;
  static uint8_t seen_pin = 0;
  static bool seen_on = false;
  called = false;
  struct Local {
    static void set_led(uint8_t pin, bool on) {
      called = true;
      seen_pin = pin;
      seen_on = on;
    }
  };
  rpc.bind("set_led", Local::set_led);

  send_request(s, 1, "set_led", 13, true);
  rpc.poll();

  CHECK(called);
  CHECK(seen_pin == 13);
  CHECK(seen_on == true);

  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  const auto pr = parse_response(frames[0]);
  REQUIRE(pr.well_formed);
  CHECK(pr.msgid == 1);
  CHECK_FALSE(pr.is_error);
}

TEST_CASE("bind: capturing lambda reaches outside state") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();

  int accumulator = 100;
  rpc.bind("add_to_accumulator", [&accumulator](int x) -> int {
    accumulator += x;
    return accumulator;
  });

  s.tx.clear();
  send_request(s, 5, "add_to_accumulator", 23);
  rpc.poll();

  CHECK(accumulator == 123);
  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  const auto pr = parse_response(frames[0]);
  REQUIRE(pr.well_formed);
  CHECK_FALSE(pr.is_error);
  Reader rr(frames[0].data() + pr.result_pos, frames[0].size() - pr.result_pos);
  int result = 0;
  rr.read(result);
  CHECK_FALSE(rr.error());
  CHECK(result == 123);
}

TEST_CASE("bind: capture-less lambda") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();

  static uint32_t period = 0;
  rpc.bind("set_period", [](uint32_t ms) { period = ms; });

  s.tx.clear();
  send_notification(s, "set_period", 250);
  rpc.poll();

  CHECK(period == 250);
  CHECK(decode_frames(s.tx).empty()); // notifications never get a reply
}

struct Motor {
  int speed = 0;
  void set_speed(int s) { speed = s; }
  int get_speed() const { return speed; }
};

TEST_CASE("bind: non-const member function") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();

  Motor motor;
  rpc.bind("set_speed", motor, &Motor::set_speed);

  s.tx.clear();
  send_request(s, 1, "set_speed", 42);
  rpc.poll();

  CHECK(motor.speed == 42);
  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  CHECK_FALSE(parse_response(frames[0]).is_error);
}

TEST_CASE("bind: const member function") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();

  Motor motor;
  motor.speed = 7;
  rpc.bind("get_speed", motor, &Motor::get_speed);

  s.tx.clear();
  send_request(s, 1, "get_speed");
  rpc.poll();

  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  const auto pr = parse_response(frames[0]);
  REQUIRE(pr.well_formed);
  CHECK_FALSE(pr.is_error);
  Reader rr(frames[0].data() + pr.result_pos, frames[0].size() - pr.result_pos);
  int result = 0;
  rr.read(result);
  CHECK(result == 7);
}

TEST_CASE("bind: map-style proxy is equivalent to bind()") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();

  rpc["doubled"] = [](int x) -> int { return x * 2; };

  s.tx.clear();
  send_request(s, 1, "doubled", 21);
  rpc.poll();

  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  const auto pr = parse_response(frames[0]);
  REQUIRE(pr.well_formed);
  CHECK_FALSE(pr.is_error);
  Reader rr(frames[0].data() + pr.result_pos, frames[0].size() - pr.result_pos);
  int result = 0;
  rr.read(result);
  CHECK(result == 42);
}

TEST_CASE("bind: const char* argument is NUL-terminated scratch-copied text") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();

  static std::string seen;
  rpc.bind("greet", [](const char *name) -> const char * {
    seen.assign(name);
    return "ack";
  });

  s.tx.clear();
  send_request(s, 1, "greet", "world");
  rpc.poll();

  CHECK(seen == "world");
  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  const auto pr = parse_response(frames[0]);
  REQUIRE(pr.well_formed);
  CHECK_FALSE(pr.is_error);
  Reader rr(frames[0].data() + pr.result_pos, frames[0].size() - pr.result_pos);
  const char *ptr;
  size_t len;
  rr.read_str(ptr, len);
  CHECK(std::string(ptr, len) == "ack");
}

// ===========================================================================
// Errors: bad args, unknown method
// ===========================================================================

TEST_CASE("dispatch: wrong argument count yields bad args") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();
  rpc.bind("add", [](int a, int b) -> int { return a + b; });

  s.tx.clear();
  send_request(s, 1, "add", 1); // only one of two args
  rpc.poll();

  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  const auto pr = parse_response(frames[0]);
  REQUIRE(pr.well_formed);
  CHECK(pr.is_error);
  CHECK(pr.error_msg == "bad args");
  CHECK(rpc.handler_errors() == 1);
}

TEST_CASE("dispatch: wrong argument type yields bad args") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();
  rpc.bind("add", [](int a, int b) -> int { return a + b; });

  s.tx.clear();
  send_request(s, 1, "add", "not a number", 2);
  rpc.poll();

  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  const auto pr = parse_response(frames[0]);
  REQUIRE(pr.well_formed);
  CHECK(pr.is_error);
  CHECK(pr.error_msg == "bad args");
}

TEST_CASE("dispatch: unknown method") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();

  s.tx.clear();
  send_request(s, 1, "nope");
  rpc.poll();

  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  const auto pr = parse_response(frames[0]);
  REQUIRE(pr.well_formed);
  CHECK(pr.is_error);
  CHECK(pr.error_msg == "unknown method");
  CHECK(rpc.unknown_method_errors() == 1);
}

// ===========================================================================
// bind_raw()
// ===========================================================================

TEST_CASE("bind_raw: handler sees Args/Reply directly") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();

  rpc.bind_raw("cfg", [](Args &a, Reply &r) {
    if (a.size() != 1) {
      r.error("bad args");
      return;
    }
    int v;
    a.reader().read(v);
    if (a.reader().error()) {
      r.error("bad args");
      return;
    }
    r.writer().pack(v * 10);
  });

  s.tx.clear();
  send_request(s, 1, "cfg", 4);
  rpc.poll();

  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  const auto pr = parse_response(frames[0]);
  REQUIRE(pr.well_formed);
  CHECK_FALSE(pr.is_error);
  Reader rr(frames[0].data() + pr.result_pos, frames[0].size() - pr.result_pos);
  int result = 0;
  rr.read(result);
  CHECK(result == 40);
}

TEST_CASE("bind_raw: a handler that writes nothing replies with a nil result") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();

  rpc.bind_raw("noop", [](Args &, Reply &) {});

  s.tx.clear();
  send_request(s, 1, "noop");
  rpc.poll();

  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  const auto pr = parse_response(frames[0]);
  REQUIRE(pr.well_formed);
  CHECK_FALSE(pr.is_error);
  Reader rr(frames[0].data() + pr.result_pos, frames[0].size() - pr.result_pos);
  CHECK(rr.type() == Reader::Type::nil);
}

// ===========================================================================
// Built-ins: rpc.ping, rpc.list, rpc.attach, rpc.detach
// ===========================================================================

TEST_CASE("rpc.ping returns the protocol version") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();

  s.tx.clear();
  send_request(s, 1, "rpc.ping");
  rpc.poll();

  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  const auto pr = parse_response(frames[0]);
  REQUIRE(pr.well_formed);
  CHECK_FALSE(pr.is_error);
  Reader rr(frames[0].data() + pr.result_pos, frames[0].size() - pr.result_pos);
  int version = 0;
  rr.read(version);
  CHECK(version == static_cast<int>(serial_rpc::kProtocolVersion));
}

TEST_CASE("rpc.list excludes built-ins and lists exactly the bound methods") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();
  rpc.bind("alpha", []() {});
  rpc.bind("beta", []() {});

  s.tx.clear();
  send_request(s, 1, "rpc.list");
  rpc.poll();

  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  const auto pr = parse_response(frames[0]);
  REQUIRE(pr.well_formed);
  CHECK_FALSE(pr.is_error);
  Reader rr(frames[0].data() + pr.result_pos, frames[0].size() - pr.result_pos);
  size_t n;
  rr.read_array(n);
  REQUIRE(n == 2);
  std::vector<std::string> names;
  for (size_t i = 0; i < n; ++i) {
    const char *p;
    size_t l;
    rr.read_str(p, l);
    names.emplace_back(p, l);
  }
  CHECK(names[0] == "alpha");
  CHECK(names[1] == "beta");
  CHECK(rpc.handler_count() == 2);
}

TEST_CASE("rpc.attach replies [version, n_methods] and attaches") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();
  rpc.bind("alpha", []() {});
  CHECK_FALSE(rpc.attached());

  s.tx.clear();
  send_request(s, 1, "rpc.attach");
  rpc.poll();

  CHECK(rpc.attached());
  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  const auto pr = parse_response(frames[0]);
  REQUIRE(pr.well_formed);
  CHECK_FALSE(pr.is_error);
  Reader rr(frames[0].data() + pr.result_pos, frames[0].size() - pr.result_pos);
  size_t n;
  rr.read_array(n);
  REQUIRE(n == 2);
  int version;
  rr.read(version);
  uint32_t n_methods;
  rr.read(n_methods);
  CHECK(version == static_cast<int>(serial_rpc::kProtocolVersion));
  CHECK(n_methods == 1);
}

TEST_CASE("rpc.detach as a notification detaches with no reply") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();
  send_request(s, 1, "rpc.attach");
  rpc.poll();
  REQUIRE(rpc.attached());

  s.tx.clear();
  send_notification(s, "rpc.detach");
  rpc.poll();

  CHECK_FALSE(rpc.attached());
  CHECK(decode_frames(s.tx).empty());
}

TEST_CASE("rpc.detach as a request replies with a nil result and detaches") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();
  send_request(s, 1, "rpc.attach");
  rpc.poll();
  REQUIRE(rpc.attached());

  s.tx.clear();
  send_request(s, 2, "rpc.detach");
  rpc.poll();

  CHECK_FALSE(rpc.attached());
  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  const auto pr = parse_response(frames[0]);
  REQUIRE(pr.well_formed);
  CHECK_FALSE(pr.is_error);
}

TEST_CASE("rpc.attach(timeout_ms) auto-detaches when no frame arrives in time") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();
  g_fake_now = 1000;
  rpc.set_clock(fake_clock);

  send_request(s, 1, "rpc.attach", 100); // 100ms timeout
  rpc.poll();
  REQUIRE(rpc.attached());

  g_fake_now = 1050; // still within the timeout
  rpc.poll();
  CHECK(rpc.attached());

  g_fake_now = 1200; // past the timeout, no frame arrived meanwhile
  rpc.poll();
  CHECK_FALSE(rpc.attached());
}

TEST_CASE("rpc.attach(timeout_ms) stays attached while frames keep arriving") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();
  g_fake_now = 0;
  rpc.set_clock(fake_clock);

  send_request(s, 1, "rpc.attach", 100);
  rpc.poll();
  REQUIRE(rpc.attached());

  for (int i = 0; i < 3; ++i) {
    g_fake_now += 60; // less than the 100ms timeout each step
    send_notification(s, "rpc.ping"); // any valid frame resets the timeout
    rpc.poll();
    CHECK(rpc.attached());
  }
}

TEST_CASE("on_attach and on_detach hooks fire") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();

  static int attach_calls = 0;
  static int detach_calls = 0;
  attach_calls = 0;
  detach_calls = 0;
  rpc.on_attach([]() { ++attach_calls; });
  rpc.on_detach([]() { ++detach_calls; });

  send_request(s, 1, "rpc.attach");
  rpc.poll();
  CHECK(attach_calls == 1);

  send_notification(s, "rpc.detach");
  rpc.poll();
  CHECK(detach_calls == 1);
}

// ===========================================================================
// notify()
// ===========================================================================

TEST_CASE("notify: returns false and sends nothing while detached") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();
  CHECK_FALSE(rpc.attached());

  s.tx.clear();
  const bool sent = rpc.notify("tick", 1);
  CHECK_FALSE(sent);
  CHECK(s.tx.empty());
}

TEST_CASE("notify: sends [2, name, [args...]] while attached") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();
  send_request(s, 1, "rpc.attach");
  rpc.poll();
  REQUIRE(rpc.attached());

  s.tx.clear();
  const bool sent = rpc.notify("tick", 7, 8);
  CHECK(sent);

  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  Reader r(frames[0].data(), frames[0].size());
  size_t n;
  r.read_array(n);
  REQUIRE(n == 3);
  int type;
  r.read(type);
  CHECK(type == 2);
  const char *p;
  size_t l;
  r.read_str(p, l);
  CHECK(std::string(p, l) == "tick");
  size_t args_n;
  r.read_array(args_n);
  REQUIRE(args_n == 2);
  int a, b;
  r.read(a);
  r.read(b);
  CHECK(a == 7);
  CHECK(b == 8);
}

// ===========================================================================
// rpc.log
// ===========================================================================

TEST_CASE("log: plain text line while detached") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();
  CHECK_FALSE(rpc.attached());

  s.tx.clear();
  rpc.log.println("hello from device");

  const auto lines = decode_text_lines(s.tx);
  REQUIRE(lines.size() == 1);
  CHECK(lines[0] == "hello from device");
  CHECK(decode_frames(s.tx).empty());
}

TEST_CASE("log: framed notification while attached") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();
  send_request(s, 1, "rpc.attach");
  rpc.poll();
  REQUIRE(rpc.attached());

  s.tx.clear();
  rpc.log_at(3).println("warn message");

  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  Reader r(frames[0].data(), frames[0].size());
  size_t n;
  r.read_array(n);
  REQUIRE(n == 3);
  int type;
  r.read(type);
  CHECK(type == 2);
  const char *p;
  size_t l;
  r.read_str(p, l);
  CHECK(std::string(p, l) == "log");
  size_t args_n;
  r.read_array(args_n);
  REQUIRE(args_n == 2);
  int level;
  r.read(level);
  CHECK(level == 3);
  r.read_str(p, l);
  CHECK(std::string(p, l) == "warn message");
}

// ===========================================================================
// on_text
// ===========================================================================

TEST_CASE("on_text: receives plain lines interleaved with frames") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();

  static std::vector<std::string> seen_lines;
  seen_lines.clear();
  rpc.on_text([](const char *line) { seen_lines.emplace_back(line); });

  s.feed_text("hello\n");
  send_request(s, 1, "rpc.ping");
  s.feed_text("world\n");
  rpc.poll();

  REQUIRE(seen_lines.size() == 2);
  CHECK(seen_lines[0] == "hello");
  CHECK(seen_lines[1] == "world");
}

// ===========================================================================
// Malformed frames
// ===========================================================================

TEST_CASE("dispatch: a frame that isn't a well-formed request/notification is counted, not crashed on") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();

  uint8_t payload[8];
  Writer w(payload, sizeof(payload));
  w.pack_array(2); // neither a request (4) nor a notification (3) shape
  w.pack(0);
  w.pack(123);
  s.feed_frame(payload, w.size());

  rpc.poll();
  CHECK(rpc.malformed_errors() == 1);
}

// ===========================================================================
// STL-only extras (this target only)
// ===========================================================================

#if SERIAL_RPC_USE_STL

TEST_CASE("STL: std::string argument and return value") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();
  rpc.bind("shout", [](std::string s) -> std::string {
    for (char &c : s) c = static_cast<char>(::toupper(static_cast<unsigned char>(c)));
    return s;
  });

  s.tx.clear();
  send_request(s, 1, "shout", "hi"); // wire encoding of a msgpack string is the same either way
  rpc.poll();

  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  const auto pr = parse_response(frames[0]);
  REQUIRE(pr.well_formed);
  CHECK_FALSE(pr.is_error);
  Reader rr(frames[0].data() + pr.result_pos, frames[0].size() - pr.result_pos);
  const char *p;
  size_t l;
  rr.read_str(p, l);
  CHECK(std::string(p, l) == "HI");
}

TEST_CASE("STL: std::vector<int> argument") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();
  rpc.bind("sum", [](std::vector<int> v) -> int {
    int total = 0;
    for (int x : v) total += x;
    return total;
  });

  uint8_t payload[64];
  Writer w(payload, sizeof(payload));
  w.pack_array(4);
  w.pack(0);
  w.pack(1);
  w.pack("sum");
  w.pack_array(1);
  w.pack_array(4);
  w.pack(1);
  w.pack(2);
  w.pack(3);
  w.pack(4);
  s.feed_frame(payload, w.size());

  s.tx.clear();
  rpc.poll();

  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  const auto pr = parse_response(frames[0]);
  REQUIRE(pr.well_formed);
  CHECK_FALSE(pr.is_error);
  Reader rr(frames[0].data() + pr.result_pos, frames[0].size() - pr.result_pos);
  int result = 0;
  rr.read(result);
  CHECK(result == 10);
}

TEST_CASE("STL: std::array<T,N> argument") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();
  rpc.bind("dot2", [](std::array<int, 2> v) -> int { return v[0] * v[1]; });

  uint8_t payload[64];
  Writer w(payload, sizeof(payload));
  w.pack_array(4);
  w.pack(0);
  w.pack(1);
  w.pack("dot2");
  w.pack_array(1);
  w.pack_array(2);
  w.pack(6);
  w.pack(7);
  s.feed_frame(payload, w.size());

  s.tx.clear();
  rpc.poll();

  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  const auto pr = parse_response(frames[0]);
  REQUIRE(pr.well_formed);
  CHECK_FALSE(pr.is_error);
  Reader rr(frames[0].data() + pr.result_pos, frames[0].size() - pr.result_pos);
  int result = 0;
  rr.read(result);
  CHECK(result == 42);
}

TEST_CASE("STL: std::pair<A,B> argument") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();
  rpc.bind("pair_len", [](std::pair<int, std::string> p) -> int {
    return p.first + static_cast<int>(p.second.size());
  });

  uint8_t payload[64];
  Writer w(payload, sizeof(payload));
  w.pack_array(4);
  w.pack(0);
  w.pack(1);
  w.pack("pair_len");
  w.pack_array(1);
  w.pack_array(2);
  w.pack(10);
  w.pack("abcd");
  s.feed_frame(payload, w.size());

  s.tx.clear();
  rpc.poll();

  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  const auto pr = parse_response(frames[0]);
  REQUIRE(pr.well_formed);
  CHECK_FALSE(pr.is_error);
  Reader rr(frames[0].data() + pr.result_pos, frames[0].size() - pr.result_pos);
  int result = 0;
  rr.read(result);
  CHECK(result == 14);
}

TEST_CASE("STL: std::tuple<Ts...> argument") {
  MockStream s;
  TestServer rpc(s);
  rpc.begin();
  rpc.bind("tuple_sum", [](std::tuple<int, int, int> t) -> int {
    return std::get<0>(t) + std::get<1>(t) + std::get<2>(t);
  });

  uint8_t payload[64];
  Writer w(payload, sizeof(payload));
  w.pack_array(4);
  w.pack(0);
  w.pack(1);
  w.pack("tuple_sum");
  w.pack_array(1);
  w.pack_array(3);
  w.pack(1);
  w.pack(2);
  w.pack(3);
  s.feed_frame(payload, w.size());

  s.tx.clear();
  rpc.poll();

  const auto frames = decode_frames(s.tx);
  REQUIRE(frames.size() == 1);
  const auto pr = parse_response(frames[0]);
  REQUIRE(pr.well_formed);
  CHECK_FALSE(pr.is_error);
  Reader rr(frames[0].data() + pr.result_pos, frames[0].size() - pr.result_pos);
  int result = 0;
  rr.read(result);
  CHECK(result == 6);
}

#endif // SERIAL_RPC_USE_STL
