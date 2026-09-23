// Tests for arduino/SerialRPC/src/serial_rpc/framing.h
//
// Covers: the CRC16/CCITT-FALSE check value, COBS encode/decode vectors
// taken from the Wikipedia "Consistent Overhead Byte Stuffing" article's
// worked examples (including the 254/255-byte run boundary), COBS
// round-trips over seeded random buffers, malformed-COBS rejection,
// frame_encode()/Demux round trips, interleaved text and frames, joining
// mid-frame, a corrupted frame falling back to text while the next frame
// still arrives, oversized frames, a partial text line split by a frame,
// CRLF handling, the LineMax == 0 variant, and the error counters.
#include <doctest/doctest.h>

#include "serial_rpc/framing.h"

#include <cstdint>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using serial_rpc::cobs_decode;
using serial_rpc::cobs_encode;
using serial_rpc::cobs_max_encoded_size;
using serial_rpc::crc16_ccitt;
using serial_rpc::Demux;
using serial_rpc::DemuxEvent;
using serial_rpc::frame_encode;
using serial_rpc::frame_max_size;
using serial_rpc::kCobsDecodeError;

namespace {

std::vector<uint8_t> encode_vec(const std::vector<uint8_t> &in) {
  std::vector<uint8_t> out(cobs_max_encoded_size(in.size()));
  const size_t n = cobs_encode(in.data(), in.size(), out.data(), out.size());
  REQUIRE(n != 0);
  out.resize(n);
  return out;
}

std::vector<uint8_t> decode_vec(const std::vector<uint8_t> &in, size_t out_cap) {
  std::vector<uint8_t> out(out_cap);
  const size_t n = cobs_decode(in.data(), in.size(), out.data(), out.size());
  REQUIRE(n != kCobsDecodeError);
  out.resize(n);
  return out;
}

bool contains_zero(const std::vector<uint8_t> &v) {
  for (uint8_t b : v)
    if (b == 0) return true;
  return false;
}

} // namespace

TEST_CASE("crc16_ccitt check value") {
  const char *s = "123456789";
  CHECK(crc16_ccitt(reinterpret_cast<const uint8_t *>(s), 9) == 0x29B1);
}

TEST_CASE("crc16_ccitt incremental matches one-shot") {
  const std::vector<uint8_t> data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  const uint16_t one_shot = crc16_ccitt(data.data(), data.size());
  const uint16_t part1 = crc16_ccitt(data.data(), 5);
  const uint16_t incremental = crc16_ccitt(data.data() + 5, data.size() - 5, part1);
  CHECK(one_shot == incremental);
}

TEST_CASE("COBS vectors from the Wikipedia COBS article") {
  CHECK(encode_vec({0x00}) == std::vector<uint8_t>{0x01, 0x01});
  CHECK(encode_vec({0x00, 0x00}) == std::vector<uint8_t>{0x01, 0x01, 0x01});
  CHECK(encode_vec({0x00, 0x11, 0x00}) == std::vector<uint8_t>{0x01, 0x02, 0x11, 0x01});
  CHECK(encode_vec({0x11, 0x22, 0x00, 0x33}) == std::vector<uint8_t>{0x03, 0x11, 0x22, 0x02, 0x33});
  CHECK(encode_vec({0x11, 0x22, 0x33, 0x44}) == std::vector<uint8_t>{0x05, 0x11, 0x22, 0x33, 0x44});
  CHECK(encode_vec({0x11, 0x00, 0x00, 0x00}) == std::vector<uint8_t>{0x02, 0x11, 0x01, 0x01, 0x01});
}

TEST_CASE("COBS: 254 non-zero bytes encodes to FF + 254 bytes, no trailing overhead byte") {
  std::vector<uint8_t> in(254);
  for (size_t i = 0; i < in.size(); ++i) in[i] = static_cast<uint8_t>(i + 1); // 0x01..0xFE

  const std::vector<uint8_t> out = encode_vec(in);
  REQUIRE(out.size() == 255); // 1 code byte + 254 data bytes, not 256
  CHECK(out[0] == 0xFF);
  CHECK(std::vector<uint8_t>(out.begin() + 1, out.end()) == in);
  CHECK_FALSE(contains_zero(out));

  CHECK(decode_vec(out, in.size()) == in);
}

TEST_CASE("COBS: 255 non-zero bytes splits into a 254-run and a 1-run") {
  std::vector<uint8_t> in(255);
  for (size_t i = 0; i < in.size(); ++i) in[i] = static_cast<uint8_t>(i + 1); // 0x01..0xFF

  const std::vector<uint8_t> out = encode_vec(in);
  REQUIRE(out.size() == 257); // FF + 254 bytes + 02 + 1 byte
  CHECK(out[0] == 0xFF);
  CHECK(out[255] == 0x02);
  CHECK(out[256] == 0xFF); // the 255th input byte, carried as ordinary data
  CHECK_FALSE(contains_zero(out));

  CHECK(decode_vec(out, in.size()) == in);
}

TEST_CASE("COBS: a leading zero followed by a 254-run also elides the trailer") {
  std::vector<uint8_t> in(255);
  in[0] = 0x00;
  for (size_t i = 1; i < in.size(); ++i) in[i] = static_cast<uint8_t>(i); // 0x01..0xFE

  const std::vector<uint8_t> out = encode_vec(in);
  REQUIRE(out.size() == 256); // 01 FF <254 bytes>
  CHECK(out[0] == 0x01);
  CHECK(out[1] == 0xFF);
  CHECK_FALSE(contains_zero(out));

  CHECK(decode_vec(out, in.size()) == in);
}

TEST_CASE("COBS: empty input encodes to a single 0x01") {
  const std::vector<uint8_t> out = encode_vec({});
  CHECK(out == std::vector<uint8_t>{0x01});
  CHECK(decode_vec(out, 0).empty());
}

TEST_CASE("COBS: encode reports 0 on insufficient capacity") {
  const std::vector<uint8_t> in = {0x11, 0x22, 0x00, 0x33};
  uint8_t out[64];
  for (size_t cap = 0; cap < 5; ++cap) {
    CHECK(cobs_encode(in.data(), in.size(), out, cap) == 0);
  }
  CHECK(cobs_encode(in.data(), in.size(), out, 5) == 5); // exact fit succeeds
}

TEST_CASE("COBS round-trip over seeded random buffers") {
  std::mt19937 rng(1234567u);
  std::uniform_int_distribution<int> len_dist(0, 600);
  std::uniform_int_distribution<int> byte_dist(0, 255);

  for (int trial = 0; trial < 200; ++trial) {
    std::vector<uint8_t> in(static_cast<size_t>(len_dist(rng)));
    for (uint8_t &b : in) b = static_cast<uint8_t>(byte_dist(rng));

    std::vector<uint8_t> encoded(cobs_max_encoded_size(in.size()));
    const size_t enc_len = cobs_encode(in.data(), in.size(), encoded.data(), encoded.size());
    REQUIRE(enc_len != 0);
    encoded.resize(enc_len);
    CHECK_FALSE(contains_zero(encoded));

    std::vector<uint8_t> decoded(in.size());
    const size_t dec_len = cobs_decode(encoded.data(), encoded.size(), decoded.data(), decoded.size());
    REQUIRE(dec_len != kCobsDecodeError);
    decoded.resize(dec_len);
    CHECK(decoded == in);
  }
}

TEST_CASE("COBS round-trip in place (out == in), as Demux relies on") {
  std::mt19937 rng(42u);
  std::uniform_int_distribution<int> byte_dist(0, 255);

  std::vector<uint8_t> in(300);
  for (uint8_t &b : in) b = static_cast<uint8_t>(byte_dist(rng));

  std::vector<uint8_t> buf(cobs_max_encoded_size(in.size()));
  const size_t enc_len = cobs_encode(in.data(), in.size(), buf.data(), buf.size());
  REQUIRE(enc_len != 0);

  const size_t dec_len = cobs_decode(buf.data(), enc_len, buf.data(), buf.size());
  REQUIRE(dec_len != kCobsDecodeError);
  buf.resize(dec_len);
  CHECK(buf == in);
}

TEST_CASE("COBS decode rejects malformed input") {
  uint8_t out[32];

  SUBCASE("embedded 0x00 inside a data run") {
    const std::vector<uint8_t> bad = {0x03, 0x11, 0x00}; // code says 2 data bytes, second is 0x00
    CHECK(cobs_decode(bad.data(), bad.size(), out, sizeof(out)) == kCobsDecodeError);
  }
  SUBCASE("code byte points past the end") {
    const std::vector<uint8_t> bad = {0x05, 0x11, 0x22}; // code says 4 data bytes, only 2 remain
    CHECK(cobs_decode(bad.data(), bad.size(), out, sizeof(out)) == kCobsDecodeError);
  }
  SUBCASE("a literal 0x00 code byte") {
    const std::vector<uint8_t> bad = {0x02, 0x11, 0x00, 0x22};
    CHECK(cobs_decode(bad.data(), bad.size(), out, sizeof(out)) == kCobsDecodeError);
  }
  SUBCASE("output capacity too small") {
    const std::vector<uint8_t> good = encode_vec({1, 2, 3, 4, 5});
    uint8_t small_out[3];
    CHECK(cobs_decode(good.data(), good.size(), small_out, sizeof(small_out)) == kCobsDecodeError);
  }
}

TEST_CASE("frame_encode/decode round trip via cobs_decode + crc16_ccitt") {
  const std::vector<uint8_t> payload = {0x00, 0x2A, 0x01, 0x02, 0x00, 0xFF};
  uint8_t frame[64];
  const size_t n = frame_encode(payload.data(), payload.size(), frame, sizeof(frame));
  REQUIRE(n != 0);
  REQUIRE(n <= frame_max_size(payload.size()));
  CHECK(frame[0] == 0x00);
  CHECK(frame[n - 1] == 0x00);
  for (size_t i = 1; i + 1 < n; ++i) CHECK(frame[i] != 0x00); // no delimiter hidden inside

  uint8_t decoded[64];
  const size_t dec_len = cobs_decode(frame + 1, n - 2, decoded, sizeof(decoded));
  REQUIRE(dec_len == payload.size() + 2);
  CHECK(std::memcmp(decoded, payload.data(), payload.size()) == 0);
  const uint16_t crc = crc16_ccitt(payload.data(), payload.size());
  CHECK(decoded[payload.size()] == static_cast<uint8_t>(crc >> 8));
  CHECK(decoded[payload.size() + 1] == static_cast<uint8_t>(crc & 0xff));
}

TEST_CASE("frame_encode reports 0 on insufficient capacity") {
  const std::vector<uint8_t> payload = {1, 2, 3, 4, 5};
  uint8_t frame[64];
  CHECK(frame_encode(payload.data(), payload.size(), frame, 0) == 0);
  CHECK(frame_encode(payload.data(), payload.size(), frame, 1) == 0);
  const size_t needed = frame_encode(payload.data(), payload.size(), frame, sizeof(frame));
  REQUIRE(needed != 0);
  CHECK(frame_encode(payload.data(), payload.size(), frame, needed - 1) == 0);
  CHECK(frame_encode(payload.data(), payload.size(), frame, needed) == needed);
}

namespace {

/// Feeds every byte of `bytes` into `demux`, and returns the events
/// produced in order as (event, text-or-payload-copy) pairs, so tests can
/// assert on the whole sequence without hand-unrolling feed() calls.
template <typename DemuxT>
struct DemuxEventLog {
  struct Entry {
    DemuxEvent event;
    std::vector<uint8_t> bytes; // payload for frame, line content for text_line
  };
  std::vector<Entry> entries;
};

template <typename DemuxT>
DemuxEventLog<DemuxT> feed_all(DemuxT &demux, const std::vector<uint8_t> &bytes) {
  DemuxEventLog<DemuxT> log;
  for (uint8_t b : bytes) {
    const DemuxEvent ev = demux.feed(b);
    if (ev == DemuxEvent::none) continue;
    typename DemuxEventLog<DemuxT>::Entry entry;
    entry.event = ev;
    entry.bytes.assign(demux.data(), demux.data() + demux.size());
    log.entries.push_back(entry);
  }
  return log;
}

std::vector<uint8_t> str_bytes(const std::string &s) { return std::vector<uint8_t>(s.begin(), s.end()); }

std::vector<uint8_t> build_frame(const std::vector<uint8_t> &payload) {
  std::vector<uint8_t> out(frame_max_size(payload.size()));
  const size_t n = frame_encode(payload.data(), payload.size(), out.data(), out.size());
  REQUIRE(n != 0);
  out.resize(n);
  return out;
}

/// Flips `f[index]` to a different, still-non-zero value. A plain XOR
/// could happen to land on 0x00, which -- inside the COBS-encoded body of
/// a frame -- is not "a corrupted byte", it is a premature frame
/// terminator, which would silently change what's being tested. COBS
/// output never contains 0x00, so `f[index]` is guaranteed non-zero
/// beforehand; this keeps it that way.
void corrupt_byte(std::vector<uint8_t> &f, size_t index) {
  uint8_t &b = f[index];
  b = static_cast<uint8_t>(b ^ 0xFF);
  if (b == 0) b = 1;
}

} // namespace

TEST_CASE("Demux: frame_encode -> Demux round trip") {
  Demux<64, 32> demux;
  const std::vector<uint8_t> payload = {0x01, 0x00, 0x7F, 0xFF};
  const std::vector<uint8_t> frame = build_frame(payload);

  DemuxEvent last = DemuxEvent::none;
  for (size_t i = 0; i + 1 < frame.size(); ++i) { // all but the last byte
    last = demux.feed(frame[i]);
    CHECK(last == DemuxEvent::none);
  }
  last = demux.feed(frame.back());
  REQUIRE(last == DemuxEvent::frame);
  CHECK(demux.size() == payload.size());
  CHECK(std::memcmp(demux.data(), payload.data(), payload.size()) == 0);
  CHECK(demux.crc_errors() == 0);
  CHECK(demux.cobs_errors() == 0);
}

TEST_CASE("Demux: plain text line") {
  Demux<64, 32> demux;
  auto log = feed_all(demux, str_bytes("hello\n"));
  REQUIRE(log.entries.size() == 1);
  CHECK(log.entries[0].event == DemuxEvent::text_line);
  CHECK(std::string(log.entries[0].bytes.begin(), log.entries[0].bytes.end()) == "hello");
  CHECK(std::string(demux.line()) == "hello");
  CHECK_FALSE(demux.truncated());
}

TEST_CASE("Demux: interleaved text and frames") {
  Demux<64, 32> demux;
  std::vector<uint8_t> stream = str_bytes("ready\n");
  const std::vector<uint8_t> f1 = build_frame({1, 2, 3});
  stream.insert(stream.end(), f1.begin(), f1.end());
  const std::vector<uint8_t> more_text = str_bytes("ok\n");
  stream.insert(stream.end(), more_text.begin(), more_text.end());
  const std::vector<uint8_t> f2 = build_frame({9, 9});
  stream.insert(stream.end(), f2.begin(), f2.end());

  auto log = feed_all(demux, stream);
  REQUIRE(log.entries.size() == 4);
  CHECK(log.entries[0].event == DemuxEvent::text_line);
  CHECK(std::string(log.entries[0].bytes.begin(), log.entries[0].bytes.end()) == "ready");
  CHECK(log.entries[1].event == DemuxEvent::frame);
  CHECK(log.entries[1].bytes == std::vector<uint8_t>{1, 2, 3});
  CHECK(log.entries[2].event == DemuxEvent::text_line);
  CHECK(std::string(log.entries[2].bytes.begin(), log.entries[2].bytes.end()) == "ok");
  CHECK(log.entries[3].event == DemuxEvent::frame);
  CHECK(log.entries[3].bytes == std::vector<uint8_t>{9, 9});
}

TEST_CASE("Demux: a frame interrupting a partial line yields frame then the joined line") {
  Demux<64, 32> demux;
  std::vector<uint8_t> stream = str_bytes("a");
  const std::vector<uint8_t> f = build_frame({0x42});
  stream.insert(stream.end(), f.begin(), f.end());
  const std::vector<uint8_t> rest = str_bytes("b\n");
  stream.insert(stream.end(), rest.begin(), rest.end());

  auto log = feed_all(demux, stream);
  REQUIRE(log.entries.size() == 2);
  CHECK(log.entries[0].event == DemuxEvent::frame);
  CHECK(log.entries[0].bytes == std::vector<uint8_t>{0x42});
  CHECK(log.entries[1].event == DemuxEvent::text_line);
  CHECK(std::string(log.entries[1].bytes.begin(), log.entries[1].bytes.end()) == "ab");
}

TEST_CASE("Demux: joining mid-frame resyncs via an empty '00 00' marker") {
  Demux<64, 32> demux;
  const std::vector<uint8_t> f = build_frame({1, 2, 3, 4});
  // Start feeding from partway through the encoded frame, as a receiver
  // attaching mid-stream would see. This first, truncated "frame" (from
  // the mid-point 0x00 boundary we happen to land on to the frame's real
  // trailing 0x00) is garbage and gets rejected; the leading 0x00 of a
  // *second*, complete frame is what actually resynchronizes.
  std::vector<uint8_t> stream(f.begin() + 2, f.end());
  const std::vector<uint8_t> f2 = build_frame({5, 6});
  stream.insert(stream.end(), f2.begin(), f2.end());

  auto log = feed_all(demux, stream);
  REQUIRE_FALSE(log.entries.empty());
  const auto &last = log.entries.back();
  CHECK(last.event == DemuxEvent::frame);
  CHECK(last.bytes == std::vector<uint8_t>{5, 6});
}

TEST_CASE("Demux: '00 00' empty frame marker resyncs with no event and no error") {
  Demux<64, 32> demux;
  CHECK(demux.feed(0x00) == DemuxEvent::none); // enter FRAME
  CHECK(demux.feed(0x00) == DemuxEvent::none); // "00 00": stay in FRAME, no error
  CHECK(demux.crc_errors() == 0);
  CHECK(demux.cobs_errors() == 0);

  const std::vector<uint8_t> f = build_frame({7, 8, 9});
  DemuxEvent last = DemuxEvent::none;
  for (size_t i = 1; i < f.size(); ++i) last = demux.feed(f[i]); // skip the leading 0x00, already consumed above
  REQUIRE(last == DemuxEvent::frame);
  CHECK(demux.data()[0] == 7);
}

TEST_CASE("Demux: a corrupted frame falls back to text and the next frame still arrives") {
  Demux<64, 32> demux;
  std::vector<uint8_t> f = build_frame({10, 20, 30});
  REQUIRE(f.size() > 3);
  corrupt_byte(f, 2); // flip a byte inside the COBS-encoded body

  const std::vector<uint8_t> f2 = build_frame({40, 50});
  std::vector<uint8_t> stream = f;
  stream.insert(stream.end(), f2.begin(), f2.end());

  auto log = feed_all(demux, stream);
  // The corrupted frame produces no event by itself (pushed back as
  // pending text, not flushed without a line terminator); the second,
  // valid frame must still be reported.
  REQUIRE_FALSE(log.entries.empty());
  const auto &last = log.entries.back();
  CHECK(last.event == DemuxEvent::frame);
  CHECK(last.bytes == std::vector<uint8_t>{40, 50});
  CHECK(demux.crc_errors() + demux.cobs_errors() >= 1);
}

TEST_CASE("Demux: rejected frame bytes are appended as text and surface once TEXT state resumes") {
  // Per docs/PLAN.md, a rejected frame's terminating 0x00 is treated as
  // the possible start of a *new* frame, so Demux deliberately stays in
  // FRAME state after a rejection -- a bare '\n' fed right afterwards
  // would just be collected as (non-terminating) frame body bytes, not
  // interpreted as text. The garbage only becomes visible as a text_line
  // once something takes Demux back to TEXT state, e.g. a subsequent
  // frame that validates successfully.
  Demux<64, 32> demux;
  std::vector<uint8_t> f = build_frame({1, 2, 3, 4, 5});
  REQUIRE(f.size() > 3);
  corrupt_byte(f, 1); // -> CRC or COBS failure

  auto log = feed_all(demux, f);
  CHECK(log.entries.empty()); // pushed back silently, no terminator yet
  CHECK(demux.crc_errors() + demux.cobs_errors() == 1);

  const std::vector<uint8_t> good = build_frame({9});
  DemuxEvent last = DemuxEvent::none;
  for (uint8_t b : good) last = demux.feed(b);
  REQUIRE(last == DemuxEvent::frame); // back in TEXT state now; pending line untouched
  CHECK(demux.data()[0] == 9);

  // Real text typed afterward joins the same pending line as the earlier
  // garbage, and both surface together on the next '\n'. Read the line
  // back through data()/size() rather than the line()/std::string route:
  // the pushed-back garbage is raw, possibly-binary bytes and is not
  // guaranteed free of embedded NULs the way genuine typed text is, and
  // size() (a tracked count) stays accurate where strlen() would not.
  CHECK(demux.feed('z') == DemuxEvent::none);
  REQUIRE(demux.feed('\n') == DemuxEvent::text_line);
  REQUIRE(demux.size() > 0);
  CHECK(demux.data()[demux.size() - 1] == 'z');
}

TEST_CASE("Demux: oversized frame is rejected and counted, byte after overflow reprocessed as text") {
  Demux<8, 32> demux; // deliberately tiny FrameMax
  const std::vector<uint8_t> payload(20, 0x55);
  const std::vector<uint8_t> f = build_frame(payload);
  REQUIRE(f.size() > 8);

  auto log = feed_all(demux, f);
  CHECK(demux.overflow_errors() == 1);
  CHECK(demux.crc_errors() == 0);
  CHECK(demux.cobs_errors() == 0);
  // Whatever came after the overflow point is now flowing through TEXT
  // state; feeding a newline should surface it as a (possibly garbled,
  // possibly truncated) text line without crashing or losing sync.
  const DemuxEvent ev = demux.feed('\n');
  CHECK((ev == DemuxEvent::text_line || ev == DemuxEvent::none));
}

TEST_CASE("Demux: overflow byte that is itself a frame delimiter starts a fresh frame") {
  Demux<4, 32> demux; // FrameMax smaller than the encoded frame below
  const std::vector<uint8_t> f1 = build_frame({1, 2, 3, 4, 5, 6});
  REQUIRE(f1.size() > 4);
  const std::vector<uint8_t> f2 = build_frame({42});

  std::vector<uint8_t> stream = f1;
  stream.insert(stream.end(), f2.begin(), f2.end());

  auto log = feed_all(demux, stream);
  CHECK(demux.overflow_errors() >= 1);
  REQUIRE_FALSE(log.entries.empty());
  CHECK(log.entries.back().event == DemuxEvent::frame);
  CHECK(log.entries.back().bytes == std::vector<uint8_t>{42});
}

TEST_CASE("Demux: partial text line split by a frame, per docs/PLAN.md's worked example") {
  Demux<64, 32> demux;
  CHECK(demux.feed('a') == DemuxEvent::none);
  const std::vector<uint8_t> f = build_frame({0x99});
  DemuxEvent ev = DemuxEvent::none;
  for (uint8_t b : f) ev = demux.feed(b);
  REQUIRE(ev == DemuxEvent::frame);
  CHECK(demux.feed('b') == DemuxEvent::none);
  REQUIRE(demux.feed('\n') == DemuxEvent::text_line);
  CHECK(std::string(demux.line()) == "ab");
}

TEST_CASE("Demux: CRLF handling") {
  SUBCASE("\\r\\n counts as one terminator") {
    Demux<64, 32> demux;
    auto log = feed_all(demux, str_bytes("line1\r\nline2\r\n"));
    REQUIRE(log.entries.size() == 2);
    CHECK(std::string(log.entries[0].bytes.begin(), log.entries[0].bytes.end()) == "line1");
    CHECK(std::string(log.entries[1].bytes.begin(), log.entries[1].bytes.end()) == "line2");
  }
  SUBCASE("bare \\n also terminates") {
    Demux<64, 32> demux;
    auto log = feed_all(demux, str_bytes("only\n"));
    REQUIRE(log.entries.size() == 1);
    CHECK(std::string(log.entries[0].bytes.begin(), log.entries[0].bytes.end()) == "only");
  }
  SUBCASE("a lone \\r not followed by \\n terminates immediately (documented choice)") {
    Demux<64, 32> demux;
    CHECK(demux.feed('x') == DemuxEvent::none);
    CHECK(demux.feed('\r') == DemuxEvent::text_line);
    CHECK(std::string(demux.line()) == "x");
    // The byte following the lone \r starts a fresh line normally.
    CHECK(demux.feed('y') == DemuxEvent::none);
    REQUIRE(demux.feed('\n') == DemuxEvent::text_line);
    CHECK(std::string(demux.line()) == "y");
  }
}

TEST_CASE("Demux: line truncation past LineMax - 1") {
  Demux<64, 4> demux; // room for 3 body chars + NUL
  auto log = feed_all(demux, str_bytes("abcdefgh\n"));
  REQUIRE(log.entries.size() == 1);
  CHECK(log.entries[0].event == DemuxEvent::text_line);
  CHECK(std::string(demux.line()) == "abc");
  CHECK(demux.truncated());
}

TEST_CASE("Demux: LineMax == 0 drops text bytes entirely and never emits text_line") {
  Demux<64, 0> demux;
  auto log = feed_all(demux, str_bytes("some text\r\nmore text\n"));
  CHECK(log.entries.empty());
  CHECK_FALSE(demux.truncated());
  CHECK(std::string(demux.line()).empty());

  // Frames still work normally.
  const std::vector<uint8_t> f = build_frame({1, 2, 3});
  DemuxEvent ev = DemuxEvent::none;
  for (uint8_t b : f) ev = demux.feed(b);
  REQUIRE(ev == DemuxEvent::frame);
  CHECK(demux.size() == 3);
  CHECK(demux.data()[0] == 1);
}

TEST_CASE("Demux: reset() clears in-progress state but not error counters") {
  Demux<64, 32> demux;
  CHECK(demux.feed('a') == DemuxEvent::none);
  std::vector<uint8_t> f = build_frame({1, 2, 3});
  corrupt_byte(f, 1);
  for (uint8_t b : f) demux.feed(b);
  CHECK(demux.crc_errors() + demux.cobs_errors() == 1);

  demux.reset();
  CHECK(demux.crc_errors() + demux.cobs_errors() == 1); // stats survive reset()
  CHECK(demux.feed('\n') == DemuxEvent::text_line);      // the pending "a..." line is gone
  CHECK(std::string(demux.line()).empty());
}
