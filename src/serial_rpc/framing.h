// SPDX-License-Identifier: Apache-2.0
/// \file framing.h
/// \brief COBS framing, CRC16 and the text/frame demultiplexer shared by
///   the Arduino device library and the host RPC client.
///
/// Hard constraints (see docs/PLAN.md, component 2, "Wire protocol"):
///  - Must compile as C++11 with avr-gcc 7.3: no STL headers (only the
///    freestanding C headers <stdint.h>, <stddef.h>), no exceptions, no
///    heap allocation, no <type_traits>.
///  - Must also compile warning-free as C++20 with Clang on the host.
///
/// A frame on the wire is `0x00 . COBS(payload || CRC16-CCITT-FALSE-BE) .
/// 0x00`. Because COBS output never contains a 0x00 byte and printable
/// text never contains one either, `Demux` can tell frames and text lines
/// apart with no separate mode switch, and resynchronizes on its own after
/// joining mid-stream or after a corrupted frame (see the class comment on
/// `Demux` below for the exact recovery rules and the design choices made
/// where the plan leaves room to choose).
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace serial_rpc {

/// Computes CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF by default, no
/// input/output reflection, xorout 0x0000) over `len` bytes starting at
/// `data`. Passing a previous call's return value as `init` computes a CRC
/// incrementally over several chunks.
///
/// Check value: crc16_ccitt(bytes of "123456789", 9) == 0x29B1.
///
/// Implemented as a plain bitwise shift-register loop (8 shifts per byte)
/// rather than the usual 256-entry lookup table, to keep flash usage
/// minimal on AVR; this code is not on a hot path.
inline uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t init = 0xFFFF) {
  uint16_t crc = init;
  for (size_t i = 0; i < len; ++i) {
    crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(data[i]) << 8));
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000u) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

/// Sentinel returned by cobs_decode() on malformed input: a literal 0x00
/// byte where only a code byte or non-zero data is valid, a code byte
/// whose data run extends past the end of `in`, or insufficient `out`
/// capacity. A legitimate decoded length can never exceed the encoded
/// input length `len` (every COBS group consumes at least as many input
/// bytes as it produces output bytes -- one code byte in, at most one
/// implicit zero out), so this all-bits-set value can never be a real
/// result and is safe to compare against with `==`.
constexpr size_t kCobsDecodeError = static_cast<size_t>(-1);

/// Worst-case number of bytes cobs_encode() can produce for `n` input
/// bytes: one overhead byte per run of up to 254 bytes, plus one leading
/// overhead byte. This is a safe upper bound for buffer sizing; the actual
/// output can be one byte shorter when a 254-byte run ends exactly at the
/// end of input (see the design note in detail::CobsStreamEncoder).
constexpr size_t cobs_max_encoded_size(size_t n) { return n + n / 254 + 1; }

namespace detail {

/// \brief Streaming COBS encoder used by both cobs_encode() and
///   frame_encode().
///
/// COBS encoding is naturally a streaming algorithm: it never needs to
/// look more than one byte ahead, *except* for one edge case -- when a run
/// of exactly 254 non-zero bytes ends exactly at the end of the whole
/// message, the trailing "next group" overhead byte that a naive
/// implementation would emit turns out to be unnecessary and is elided by
/// every reference COBS implementation (confirmed against the worked
/// examples in the Wikipedia "Consistent Overhead Byte Stuffing" article,
/// e.g. encoding exactly 254 non-zero bytes yields `FF <254 bytes>`, not
/// `FF <254 bytes> 01`). Knowing whether a given byte is the *last* byte
/// of the whole message is therefore required, not just "is more data
/// available in the current call".
///
/// To make this work across multiple feed() calls without buffering the
/// whole message first (the point of streaming this at all -- see
/// frame_encode(), which streams payload bytes followed by 2 CRC bytes
/// without ever concatenating them into one buffer), each feed() call
/// takes an explicit `more_follows` flag: false only on the call that
/// supplies the last bytes of the message. Getting this flag right is the
/// caller's responsibility; cobs_encode() itself makes a single feed()
/// call with more_follows = false.
class CobsStreamEncoder {
public:
  CobsStreamEncoder(uint8_t *out, size_t cap) : _out(out), _cap(cap), _encode(1), _codep(0), _code(1), _ok(true) {}

  /// Feeds `n` bytes. Set `more_follows` to false only if this is the
  /// final feed() call before finish(). Returns false once capacity has
  /// been exceeded (sticky: every further call also returns false).
  bool feed(const uint8_t *data, size_t n, bool more_follows) {
    if (!_ok) return false;
    for (size_t i = 0; i < n; ++i) {
      const bool more_remaining = more_follows || (i + 1 < n);
      if (!put_one_(data[i], more_remaining)) return false;
    }
    return true;
  }

  /// Flushes the pending code byte. Call once after the last feed().
  /// Returns the total encoded length, or 0 if capacity was exceeded at
  /// any point (encoding a real, possibly-empty message never legitimately
  /// produces 0 bytes -- even an empty message encodes to one byte, `01`).
  size_t finish() {
    if (!_ok) return 0;
    // If the last group was opened by the 254-byte-cap rollover on the
    // very last byte of the message (the edge case described above),
    // `_codep` was deliberately left equal to `_encode` (no slot
    // reserved) and there is nothing left to flush.
    if (_codep < _encode) {
      if (_codep >= _cap) {
        _ok = false;
        return 0;
      }
      _out[_codep] = _code;
    }
    return _encode;
  }

private:
  bool put_one_(uint8_t byte, bool more_remaining) {
    if (byte != 0) {
      if (_encode >= _cap) {
        _ok = false;
        return false;
      }
      _out[_encode] = byte;
      ++_encode;
      ++_code;
    }
    if (byte == 0 || _code == 0xff) {
      if (_codep >= _cap) {
        _ok = false;
        return false;
      }
      _out[_codep] = _code;
      _code = 1;
      _codep = _encode;
      // Reserve the next code-byte slot, unless this is a 254-cap
      // rollover (byte != 0) with nothing left to encode: eliding the
      // advance here is exactly what makes finish() skip the trailing
      // byte in that case.
      if (byte == 0 || more_remaining) {
        ++_encode;
      }
    }
    return true;
  }

  uint8_t *_out;
  size_t _cap;
  size_t _encode; // next free output index
  size_t _codep;  // index of the pending (not yet written) code byte
  uint8_t _code;  // value of the pending code byte
  bool _ok;
};

} // namespace detail

/// Encodes `len` bytes at `in` as a COBS block (no leading/trailing frame
/// delimiter, no CRC -- just the codec) into `out`. `out` never contains a
/// 0x00 byte. Returns the number of bytes written, or 0 if `out_cap` is
/// too small.
inline size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap) {
  detail::CobsStreamEncoder enc(out, out_cap);
  if (!enc.feed(in, len, false)) return 0;
  return enc.finish();
}

/// Decodes a COBS block (as produced by cobs_encode(), with no embedded
/// 0x00 and no frame delimiters) back into `out`. `in` and `out` may be
/// the same buffer: decoding a valid COBS block never writes to an index
/// past the read cursor's current position (each group consumes its code
/// byte plus `code - 1` data bytes from `in` while producing at most
/// `code - 1` data bytes plus one implicit zero into `out`, so the write
/// cursor never catches up with the read cursor), so in-place decoding is
/// always memory-safe. `Demux` relies on this to decode a just-terminated
/// frame without a second buffer.
///
/// Returns the decoded length, or kCobsDecodeError if `in` contains a
/// literal 0x00 byte, a code byte whose run runs past the end of `in`, or
/// the decoded output would not fit in `out_cap`.
inline size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap) {
  size_t read_index = 0;
  size_t write_index = 0;
  while (read_index < len) {
    const uint8_t code = in[read_index];
    if (code == 0) return kCobsDecodeError; // 0x00 is never valid inside a COBS block
    ++read_index;
    const size_t data_len = static_cast<size_t>(code - 1);
    if (data_len > len - read_index) return kCobsDecodeError; // code points past the end
    for (size_t i = 0; i < data_len; ++i) {
      const uint8_t b = in[read_index + i];
      if (b == 0) return kCobsDecodeError; // embedded 0x00 where only non-zero data is valid
      if (write_index >= out_cap) return kCobsDecodeError;
      out[write_index++] = b;
    }
    read_index += data_len;
    // A code of 0xff means this group was split by the 254-byte cap, not
    // by a real zero byte in the original data, so no zero is inserted
    // here; likewise the very last group in the block never gets a
    // trailing zero (there is nothing after it).
    if (code != 0xff && read_index < len) {
      if (write_index >= out_cap) return kCobsDecodeError;
      out[write_index++] = 0;
    }
  }
  return write_index;
}

/// Worst-case total size of a frame_encode() output for an `n`-byte
/// payload: the two 0x00 delimiters plus the worst-case COBS size of the
/// payload with its 2-byte CRC appended.
constexpr size_t frame_max_size(size_t n) { return cobs_max_encoded_size(n + 2) + 2; }

/// Produces `0x00 . COBS(payload || CRC16-CCITT-FALSE-BE) . 0x00` into
/// `out`. The payload and the 2 big-endian CRC bytes are streamed through
/// one COBS encoder pass (see detail::CobsStreamEncoder) with no
/// intermediate buffer holding payload + CRC concatenated. Returns the
/// total number of bytes written (including both delimiters), or 0 if
/// `out_cap` is too small.
inline size_t frame_encode(const uint8_t *payload, size_t len, uint8_t *out, size_t out_cap) {
  if (out_cap < 2) return 0; // room for at least the two delimiters
  out[0] = 0x00;

  const uint16_t crc = crc16_ccitt(payload, len);
  const uint8_t crc_be[2] = {static_cast<uint8_t>((crc >> 8) & 0xffu), static_cast<uint8_t>(crc & 0xffu)};

  // Leave room for both the leading 0x00 (already written) and the
  // trailing 0x00 (written below) when bounding the COBS encoder.
  detail::CobsStreamEncoder enc(out + 1, out_cap - 2);
  if (!enc.feed(payload, len, true)) return 0;
  if (!enc.feed(crc_be, 2, false)) return 0;
  const size_t cobs_len = enc.finish();
  if (cobs_len == 0) return 0;

  out[1 + cobs_len] = 0x00;
  return cobs_len + 2;
}

namespace detail {

/// Outcome of validating one just-terminated frame's collected bytes.
enum class FrameOutcome { empty, ready, cobs_error, crc_error };

/// Decodes and CRC-validates a just-terminated frame's `len` collected
/// (still COBS-encoded) bytes, held in `buf` with physical capacity `cap`.
/// Decoding happens in place (see cobs_decode()'s in-place safety note).
///
///  - `empty`: `len == 0` (an "00 00" empty-frame marker in the wire
///    protocol). `buf` is untouched; the caller stays in FRAME state.
///  - `ready`: valid frame. The payload (CRC stripped) occupies
///    `buf[0, *out_len)`.
///  - `cobs_error` / `crc_error`: invalid frame. `buf[0, *out_len)` holds
///    bytes worth reporting back as text (see the push-back design note on
///    Demux); for a COBS failure this is the original `len` collected
///    bytes (decoding may have partially overwritten a prefix of them in
///    place before detecting the error -- this is only ever used for
///    best-effort diagnostic text, so that is an accepted trade-off); for
///    a CRC failure it is the `decoded` (payload + CRC) bytes, since COBS
///    decoding itself fully succeeded in that case.
inline FrameOutcome decode_and_validate_frame(uint8_t *buf, size_t len, size_t cap, size_t *out_len) {
  if (len == 0) {
    *out_len = 0;
    return FrameOutcome::empty;
  }
  const size_t decoded = cobs_decode(buf, len, buf, cap);
  if (decoded == kCobsDecodeError) {
    *out_len = len;
    return FrameOutcome::cobs_error;
  }
  if (decoded < 2) { // too short to carry a CRC16
    *out_len = decoded;
    return FrameOutcome::crc_error;
  }
  const size_t payload_len = decoded - 2;
  const uint16_t received =
      static_cast<uint16_t>((static_cast<uint16_t>(buf[payload_len]) << 8) | buf[payload_len + 1]);
  const uint16_t computed = crc16_ccitt(buf, payload_len);
  if (computed != received) {
    *out_len = decoded;
    return FrameOutcome::crc_error;
  }
  *out_len = payload_len;
  return FrameOutcome::ready;
}

} // namespace detail

/// Events reported by Demux::feed(). A free enum (rather than a type
/// nested in the Demux template) so it is the same type across every
/// Demux<FrameMax, LineMax> instantiation.
enum class DemuxEvent { none, text_line, frame };

/// \brief Byte-at-a-time state machine that separates plain text lines
///   from COBS/CRC frames on one shared byte stream, per docs/PLAN.md's
///   "Wire protocol" recovery rules. No heap, no exceptions.
///
/// `feed(b)` consumes one byte and returns `DemuxEvent::text_line` or
/// `::frame` when one is complete, or `::none` otherwise. `data()`/
/// `size()` describe whichever event was returned by the *last* feed()
/// call that returned a non-none event, and stay valid only until the
/// next feed() call. For a `text_line` event `data()` is also available,
/// NUL-terminated, through `line()`.
///
/// ## State machine
/// - **TEXT**: bytes accumulate into a pending line buffer.
///   - `\n` ends the line (see CRLF handling below).
///   - `\r` also ends the line immediately (it is *not* included in the
///     emitted text), and arms a one-byte "swallow the next `\n`" flag so
///     a `\r\n` pair produces exactly one text_line event, not two. If the
///     byte following a lone `\r` is anything other than `\n`, that flag
///     simply expires without effect and the byte starts a new line
///     normally. This is the documented choice for the "your choice"
///     lone-`\r` case in docs/PLAN.md.
///   - A byte longer than `LineMax - 1` characters: once the pending line
///     reaches capacity, further body bytes are silently dropped (not
///     buffered) until the next line terminator, `truncated()` reports
///     true for that line once it is emitted, and the emitted text is the
///     first `LineMax - 1` bytes actually received. This is the "emit a
///     truncated line, discard the rest" option from docs/PLAN.md.
///   - `0x00` switches to FRAME. Any partial pending line is *not*
///     cleared: it is only ever cleared when a line-ending byte is seen,
///     so `print("a"); <frame>; println("b")` yields a frame event
///     followed by a text_line event for "ab", per docs/PLAN.md.
/// - **FRAME**: raw (still COBS-encoded) bytes accumulate.
///   - An empty frame (`00 00`) leaves FRAME state unchanged and reports
///     no event -- this is what lets a receiver that joins mid-frame
///     resynchronize within at most one frame.
///   - On `0x00`, the collected bytes are COBS-decoded and CRC-checked in
///     place.
///     - Valid: reports `frame` (payload via data()/size()) and returns to
///       TEXT.
///     - Invalid (bad COBS or bad CRC): the bytes are pushed back as text
///       (see the design note below), an error counter is bumped, and
///       this `0x00` is treated as the possible start of a new frame
///       (FRAME state is kept, with the collector reset).
///   - Growing past `FrameMax` bytes without seeing a terminator: this is
///     treated the same as an invalid frame (pushed back as text,
///     `overflow_errors()` bumped), except the byte that caused the
///     overflow was never part of the collected block and is instead
///     re-fed through TEXT-state handling immediately (it might itself be
///     `\n`, `\r`, or a fresh `0x00`).
///
/// ## Push-back-as-text design
/// docs/PLAN.md leaves the exact recovery shape open ("pick something
/// correct and simple, document it, test it") because only one event can
/// be returned per feed() call, yet the garbage from a rejected frame
/// could in principle contain several lines' worth of bytes. The choice
/// made here: rejected frame bytes are appended to the pending TEXT line
/// buffer *verbatim*, through the same capacity/truncation path as normal
/// typed text, *without* scanning them for embedded `\n`/`\r` -- they
/// become literal characters of whatever line is currently pending, and
/// that line is only emitted (as one text_line event) when a real line
/// terminator is next seen in the stream. This never silently drops a
/// completed line (unlike "only the last line survives"), needs no queue
/// or extra buffer, and is simple to reason about and test. The one
/// exception is the byte that triggers a FrameMax overflow: it was never
/// "collected" as part of the rejected frame, so it gets full normal
/// TEXT-state treatment (including terminator detection) rather than a
/// silent append.
///
/// FrameMax must be large enough for the worst-case COBS-encoded size of
/// the largest expected payload plus 2 CRC bytes (see frame_max_size()).
template <size_t FrameMax, size_t LineMax>
class Demux {
  static_assert(FrameMax > 0, "Demux requires FrameMax > 0");
  static_assert(LineMax > 0, "use the Demux<FrameMax, 0> specialization for LineMax == 0");

public:
  using Event = DemuxEvent;

  Demux()
      : _state(State::text), _frame_len(0), _frame_size(0), _line_len(0), _last_line_len(0), _line_truncated(false),
        _last_line_truncated(false), _pending_cr(false), _last_was_frame(false), _crc_errors(0), _cobs_errors(0),
        _overflow_errors(0) {
    _line_buf[0] = '\0';
  }

  /// Feeds one byte and returns the event it completed, if any.
  Event feed(uint8_t b) {
    if (_state == State::text) return feed_text_(b);
    return feed_frame_(b);
  }

  /// Payload bytes (for a `frame` event) or line bytes (for a `text_line`
  /// event), valid until the next feed() call.
  const uint8_t *data() const { return _last_was_frame ? _frame_buf : reinterpret_cast<const uint8_t *>(_line_buf); }
  /// Length matching data(): payload length for `frame`, line length
  /// (excluding the NUL terminator) for `text_line`.
  size_t size() const { return _last_was_frame ? _frame_size : _last_line_len; }
  /// NUL-terminated view of the most recently emitted text line,
  /// regardless of whether a frame has been reported since (mirroring how
  /// a frame does not disturb a pending partial line).
  const char *line() const { return _line_buf; }
  /// True if the most recently emitted text line was longer than
  /// `LineMax - 1` bytes and was truncated.
  bool truncated() const { return _last_line_truncated; }

  uint32_t crc_errors() const { return _crc_errors; }
  uint32_t cobs_errors() const { return _cobs_errors; }
  uint32_t overflow_errors() const { return _overflow_errors; }

  /// Clears in-progress parse state (current state, partial line, partial
  /// frame) and returns to TEXT. Error counters are cumulative statistics
  /// and are intentionally left untouched.
  void reset() {
    _state = State::text;
    _frame_len = 0;
    _frame_size = 0;
    _line_len = 0;
    _last_line_len = 0;
    _line_truncated = false;
    _last_line_truncated = false;
    _pending_cr = false;
    _last_was_frame = false;
    _line_buf[0] = '\0';
  }

private:
  enum class State { text, frame };

  Event feed_text_(uint8_t b) {
    if (b == 0x00) {
      _state = State::frame;
      _frame_len = 0;
      return Event::none;
    }
    if (b == '\r') {
      const Event ev = emit_line_();
      _pending_cr = true;
      return ev;
    }
    if (b == '\n') {
      if (_pending_cr) {
        _pending_cr = false;
        return Event::none;
      }
      return emit_line_();
    }
    _pending_cr = false;
    append_line_byte_(b);
    return Event::none;
  }

  Event feed_frame_(uint8_t b) {
    if (b != 0x00) {
      if (_frame_len >= FrameMax) {
        ++_overflow_errors;
        push_back_frame_as_text_(_frame_len);
        _state = State::text;
        _frame_len = 0;
        return feed_text_(b); // this byte was never part of the rejected block
      }
      _frame_buf[_frame_len++] = b;
      return Event::none;
    }

    size_t out_len = 0;
    const detail::FrameOutcome outcome = detail::decode_and_validate_frame(_frame_buf, _frame_len, FrameMax, &out_len);
    switch (outcome) {
      case detail::FrameOutcome::empty:
        _frame_len = 0;
        return Event::none;
      case detail::FrameOutcome::ready:
        _frame_size = out_len;
        _last_was_frame = true;
        _state = State::text;
        return Event::frame;
      case detail::FrameOutcome::cobs_error:
        ++_cobs_errors;
        break;
      case detail::FrameOutcome::crc_error:
        ++_crc_errors;
        break;
    }
    push_back_frame_as_text_(out_len);
    _frame_len = 0; // this 0x00 may be the start of a new frame
    return Event::none;
  }

  void append_line_byte_(uint8_t b) {
    if (_line_len + 1 >= LineMax) { // no room for this byte plus the NUL
      _line_truncated = true;
      return;
    }
    _line_buf[_line_len++] = static_cast<char>(b);
  }

  Event emit_line_() {
    _line_buf[_line_len] = '\0';
    _last_line_len = _line_len;
    _last_line_truncated = _line_truncated;
    _line_len = 0;
    _line_truncated = false;
    _last_was_frame = false;
    return Event::text_line;
  }

  void push_back_frame_as_text_(size_t len) {
    for (size_t i = 0; i < len; ++i) append_line_byte_(_frame_buf[i]);
  }

  State _state;
  uint8_t _frame_buf[FrameMax];
  size_t _frame_len;  // bytes collected in FRAME state so far
  size_t _frame_size; // decoded payload length of the last `frame` event
  char _line_buf[LineMax];
  size_t _line_len;             // bytes accumulated for the in-progress line
  size_t _last_line_len;        // length of the last emitted line
  bool _line_truncated;         // in-progress line exceeded capacity
  bool _last_line_truncated;    // snapshot for truncated()
  bool _pending_cr;             // a lone '\r' is holding, waiting to see if '\n' follows
  bool _last_was_frame;         // selects data()/size() source
  uint32_t _crc_errors;
  uint32_t _cobs_errors;
  uint32_t _overflow_errors;
};

/// Specialization for `LineMax == 0`: text bytes are dropped entirely and
/// no line buffer is allocated at all (not even a 1-byte one), per
/// docs/PLAN.md. `text_line` events are never produced, `truncated()`
/// always reports false, and `line()` returns a static empty string.
/// Frame handling is identical to the primary template (duplicated here
/// rather than factored into a shared base, since the two variants'
/// public data()/size()/line() semantics differ enough -- text vs.
/// frame-only -- that sharing state through a base class would need more
/// machinery than the small amount of duplicated frame-handling code it
/// would save).
template <size_t FrameMax>
class Demux<FrameMax, 0> {
  static_assert(FrameMax > 0, "Demux requires FrameMax > 0");

public:
  using Event = DemuxEvent;

  Demux() : _state(State::text), _frame_len(0), _frame_size(0), _crc_errors(0), _cobs_errors(0), _overflow_errors(0) {}

  Event feed(uint8_t b) {
    if (_state == State::text) return feed_text_(b);
    return feed_frame_(b);
  }

  /// Payload bytes of the last `frame` event, valid until the next
  /// feed() call. Never reflects text, since text_line is never emitted.
  const uint8_t *data() const { return _frame_buf; }
  size_t size() const { return _frame_size; }
  const char *line() const { return ""; }
  bool truncated() const { return false; }

  uint32_t crc_errors() const { return _crc_errors; }
  uint32_t cobs_errors() const { return _cobs_errors; }
  uint32_t overflow_errors() const { return _overflow_errors; }

  void reset() {
    _state = State::text;
    _frame_len = 0;
    _frame_size = 0;
  }

private:
  enum class State { text, frame };

  Event feed_text_(uint8_t b) {
    if (b == 0x00) {
      _state = State::frame;
      _frame_len = 0;
    }
    // Every other byte is dropped: no line buffer exists to hold it.
    return Event::none;
  }

  Event feed_frame_(uint8_t b) {
    if (b != 0x00) {
      if (_frame_len >= FrameMax) {
        ++_overflow_errors;
        _state = State::text;
        _frame_len = 0;
        return feed_text_(b);
      }
      _frame_buf[_frame_len++] = b;
      return Event::none;
    }

    size_t out_len = 0;
    const detail::FrameOutcome outcome = detail::decode_and_validate_frame(_frame_buf, _frame_len, FrameMax, &out_len);
    switch (outcome) {
      case detail::FrameOutcome::empty:
        _frame_len = 0;
        return Event::none;
      case detail::FrameOutcome::ready:
        _frame_size = out_len;
        _state = State::text;
        return Event::frame;
      case detail::FrameOutcome::cobs_error:
        ++_cobs_errors;
        break;
      case detail::FrameOutcome::crc_error:
        ++_crc_errors;
        break;
    }
    _frame_len = 0;
    return Event::none;
  }

  State _state;
  uint8_t _frame_buf[FrameMax];
  size_t _frame_len;
  size_t _frame_size;
  uint32_t _crc_errors;
  uint32_t _cobs_errors;
  uint32_t _overflow_errors;
};

} // namespace serial_rpc
