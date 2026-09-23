/// \file server.h
/// \brief Device-side MsgPack-RPC dispatcher (see docs/PLAN.md, components 3
///   and 3b): a fixed handler table with `InplaceFn` callables on AVR / when
///   forced, or a `std::vector` + `std::function` table on 32-bit cores.
///
/// Hard constraints on the fixed backend (see docs/PLAN.md, component 3):
///  - Must compile as C++11 with avr-gcc 7.3: no STL headers, no exceptions,
///    no heap allocation. Only the freestanding C headers <stdint.h>,
///    <stddef.h>, <string.h> are used. AVR has no <new>, so placement new
///    is declared locally, guarded against a core that already has one
///    (see the "placement new" section below).
///  - Must also compile warning-free as C++20 with Clang on the host, which
///    is how both backends are exercised by tests/test_server.cpp (built
///    twice: once with SERIAL_RPC_USE_STL=0, once with =1).
///
/// The STL backend (docs/PLAN.md 3b) may use <functional>, <vector>,
/// <string>, <array>, <tuple>, <utility>, <type_traits>, and must compile
/// with -fno-exceptions under gnu++11/14/17. Both backends share the same
/// public API; the `#if SERIAL_RPC_USE_STL` blocks are kept few and local,
/// per docs/PLAN.md's implementation note for this component.
#pragma once

#include "config.h"
#include "framing.h"
#include "msgpack_lite.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#if defined(ARDUINO)
#include <Arduino.h>
#endif

#if SERIAL_RPC_USE_STL
#include <array>
#include <functional>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>
#endif

// ---------------------------------------------------------------------------
// Placement new.
//
// AVR has no <new> in the bare avr-gcc install (see docs/PLAN.md and the
// InplaceFn note below): `serial_rpc::detail::InplaceFn` needs placement new
// to construct a callable inside its fixed storage. Some AVR cores (e.g. the
// official Arduino AVR core, in cores/arduino/new + new.cpp) *do* provide
// one, but nothing in Arduino.h pulls it in by default, and third-party or
// older AVR cores may not have it at all. So: declare it ourselves, but only
// on AVR, and only if nothing has already brought in a `<new>`-compatible
// declaration into this translation unit (checked via the header guards
// used by libstdc++'s <new> and by the Arduino AVR core's own new/new.h).
// As an `inline` function this has weak linkage, so if a *strong*
// definition exists elsewhere at link time (e.g. cores/arduino/new.cpp),
// the linker silently keeps that one instead -- no duplicate-symbol error
// either way. Off AVR, a real <new> is assumed to exist and is included
// normally.
#if defined(__AVR__)
#if !defined(_NEW) && !defined(_NEW_) && !defined(NEW_H) && !defined(__NEW_H) && !defined(_GLIBCXX_NEW)
inline void *operator new(size_t, void *ptr) noexcept { return ptr; }
#endif
#else
#include <new>
#endif

// ---------------------------------------------------------------------------
// Trivially-destructible / trivially-copyable checks without <type_traits>.
//
// avr-gcc 7.3 does not have the (newer) __is_trivially_destructible /
// __is_trivially_copyable builtins, only the older __has_trivial_destructor
// / __has_trivial_copy; Clang has both pairs, but warns (-Wdeprecated-
// builtins, part of -Wall) on the older ones. So: pick whichever pair each
// compiler is happy with.
#if defined(__clang__)
#define SERIAL_RPC_IS_TRIVIALLY_DESTRUCTIBLE_(T) __is_trivially_destructible(T)
#define SERIAL_RPC_IS_TRIVIALLY_COPYABLE_(T) __is_trivially_copyable(T)
#else
#define SERIAL_RPC_IS_TRIVIALLY_DESTRUCTIBLE_(T) __has_trivial_destructor(T)
#define SERIAL_RPC_IS_TRIVIALLY_COPYABLE_(T) __has_trivial_copy(T)
#endif

namespace serial_rpc {

// ===========================================================================
// Args / Reply: what a raw handler (bind_raw) and every typed thunk see.
// ===========================================================================

/// \brief View over one request's or notification's decoded parameter
///   array, handed to raw handlers (`bind_raw`) and used internally by the
///   typed `bind()` thunks.
///
/// Wraps the shared `msgpack::Reader`, positioned right after the params
/// array header (i.e. `reader()` is ready to read `size()` elements in
/// order), plus a small bump-allocated scratch buffer used to give
/// `const char*` string arguments a NUL terminator (see the string-argument
/// design note on `ArgDecoder<const char*>` below).
class Args {
public:
  Args(msgpack::Reader &reader, size_t n, char *scratch, size_t scratch_cap)
      : _reader(reader), _n(n), _scratch(scratch), _scratch_cap(scratch_cap), _scratch_used(0) {}

  /// Number of elements in the params array.
  size_t size() const { return _n; }
  /// The shared decoder, positioned at the next undecoded parameter.
  msgpack::Reader &reader() { return _reader; }

  /// Claims `len` bytes from the shared per-call string scratch buffer, or
  /// returns nullptr if it doesn't fit. Used by ArgDecoder<const char*>.
  char *scratch_alloc(size_t len) {
    if (_scratch_used + len > _scratch_cap) return nullptr;
    char *p = _scratch + _scratch_used;
    _scratch_used += len;
    return p;
  }

private:
  msgpack::Reader &_reader;
  size_t _n;
  char *_scratch;
  size_t _scratch_cap;
  size_t _scratch_used;
};

/// \brief Where a handler (raw or typed) writes its result, and how it
///   reports failure.
///
/// `writer()` is a `msgpack::Writer` over a small private scratch buffer
/// holding *only* the encoded result value (not the whole `[1, msgid, ...]`
/// response array); the caller (`Server::send_response_`) splices those
/// bytes into the response after seeing whether `failed()`. A handler that
/// writes nothing is treated as returning nil.
class Reply {
public:
  explicit Reply(msgpack::Writer &writer) : _writer(writer), _error(nullptr) {}

  msgpack::Writer &writer() { return _writer; }
  const msgpack::Writer &writer() const { return _writer; }

  /// Sets the error slot. `msg` must outlive the response being built (a
  /// string literal, as in every example in docs/PLAN.md, always does).
  void error(const char *msg) { _error = msg; }
  const char *error_message() const { return _error; }
  bool failed() const { return _error != nullptr; }

private:
  msgpack::Writer &_writer;
  const char *_error;
};

namespace detail {

// ===========================================================================
// InplaceFn<Capacity>: a fixed-capacity, no-heap type-erased callable.
// ===========================================================================

/// Alignment helper: `InplaceFn`'s storage is aligned as strictly as the
/// strictest of these, which covers every callable this library ever
/// stores (object pointers, function pointers, member-function pointers,
/// and the scalar members a capturing lambda might hold).
union MaxAlign {
  void *p;
  void (*fp)();
  double d;
  long long ll;
};

/// \brief A small `std::function`-like type-erased callable with a fixed,
///   inline `Capacity`-byte buffer and no heap allocation (docs/PLAN.md,
///   component 3).
///
/// Only usable with callables that are trivially destructible *and*
/// trivially copyable (enforced by `static_assert`, since `InplaceFn`
/// itself relies on the compiler-generated copy/move/destroy operations
/// being plain byte copies / no-ops -- this holds for every supported
/// callable shape here: function pointers, object+member-function-pointer
/// pairs, and lambdas that only capture by reference or by trivial value).
/// Handlers are never removed once bound, so a non-trivial destructor
/// would leak with no way to run it -- hence the destructibility check.
///
/// Capacity's default (chosen by `Server`, see docs/PLAN.md 3b) is
/// `2*sizeof(void*) + sizeof(member-function pointer)`: enough for a
/// `[this]`/`[&x]`-style capture, or an object pointer plus a
/// member-function pointer.
template <size_t Capacity>
class InplaceFn {
public:
  InplaceFn() : _invoke(nullptr) {}

  template <class Fn>
  InplaceFn(Fn f) : _invoke(&invoke_impl_<Fn>) { // NOLINT(*-explicit-constructor): implicit by design, see bind()
    static_assert(sizeof(Fn) <= Capacity,
                  "serial_rpc: this callable is too large for InplaceFn's Capacity; raise Server's Capacity "
                  "template parameter, or capture by reference instead of by value");
    static_assert(alignof(Fn) <= alignof(MaxAlign),
                  "serial_rpc: this callable is over-aligned for InplaceFn's storage");
    static_assert(SERIAL_RPC_IS_TRIVIALLY_DESTRUCTIBLE_(Fn),
                  "serial_rpc: a bound callable must be trivially destructible (handlers are never destroyed on "
                  "the fixed backend); capture by reference or by pointer instead of owning a non-trivial object");
    static_assert(SERIAL_RPC_IS_TRIVIALLY_COPYABLE_(Fn),
                  "serial_rpc: a bound callable must be trivially copyable");
    ::new (static_cast<void *>(_storage)) Fn(f);
  }

  void operator()(Args &args, Reply &reply) const { _invoke(_storage, args, reply); }

  bool valid() const { return _invoke != nullptr; }

private:
  template <class Fn>
  static void invoke_impl_(const void *storage, Args &args, Reply &reply) {
    (*reinterpret_cast<const Fn *>(storage))(args, reply);
  }

  alignas(MaxAlign) unsigned char _storage[Capacity];
  void (*_invoke)(const void *, Args &, Reply &);
};

// ===========================================================================
// ArgDecoder<T> / ValuePacker<T>: the common (fixed + STL) scalar types.
// ===========================================================================

/// Primary template: decoding an unsupported type is a compile error with a
/// readable message, not a template-instantiation cascade. `sizeof(T) == 0`
/// is always false once T is a complete type; it only fires when this
/// primary template is actually instantiated (i.e. no specialization
/// matched T), which is exactly the "unsupported type" case.
template <class T>
struct ArgDecoder {
  static bool decode(Args &, T &) {
    static_assert(sizeof(T) == 0,
                  "serial_rpc: this argument type is not supported by bind(); supported types are bool, the "
                  "fundamental integer types, float, double and const char* everywhere, plus std::string, "
                  "std::vector<T>, std::array<T,N>, std::pair and std::tuple when SERIAL_RPC_USE_STL=1. Use "
                  "bind_raw() to decode anything else yourself.");
    return false;
  }
};

/// Primary template for packing a handler's return value; see ArgDecoder<T>
/// above for the same "unsupported type" trick.
template <class T>
struct ValuePacker {
  static void pack(msgpack::Writer &w, const T &v) { w.pack(v); }
};

#define SERIAL_RPC_SCALAR_ARG_DECODER_(T)                                                                            \
  template <>                                                                                                        \
  struct ArgDecoder<T> {                                                                                             \
    static bool decode(Args &args, T &out) {                                                                        \
      args.reader().read(out);                                                                                      \
      return !args.reader().error();                                                                                \
    }                                                                                                                \
  };

SERIAL_RPC_SCALAR_ARG_DECODER_(bool)
SERIAL_RPC_SCALAR_ARG_DECODER_(signed char)
SERIAL_RPC_SCALAR_ARG_DECODER_(short) // NOLINT(*-runtime-int)
SERIAL_RPC_SCALAR_ARG_DECODER_(int)
SERIAL_RPC_SCALAR_ARG_DECODER_(long) // NOLINT(*-runtime-int)
SERIAL_RPC_SCALAR_ARG_DECODER_(long long) // NOLINT(*-runtime-int)
SERIAL_RPC_SCALAR_ARG_DECODER_(unsigned char)
SERIAL_RPC_SCALAR_ARG_DECODER_(unsigned short) // NOLINT(*-runtime-int)
SERIAL_RPC_SCALAR_ARG_DECODER_(unsigned int)
SERIAL_RPC_SCALAR_ARG_DECODER_(unsigned long) // NOLINT(*-runtime-int)
SERIAL_RPC_SCALAR_ARG_DECODER_(unsigned long long) // NOLINT(*-runtime-int)
SERIAL_RPC_SCALAR_ARG_DECODER_(float)
SERIAL_RPC_SCALAR_ARG_DECODER_(double)

#undef SERIAL_RPC_SCALAR_ARG_DECODER_

/// \brief Zero-copy-decode, then NUL-terminate into the per-call scratch
///   buffer.
///
/// `msgpack::Reader::read_str` is zero-copy: it hands back a pointer
/// straight into the (COBS-decoded) receive frame, with a separate length,
/// not a NUL-terminated C string, and the bytes right after it in that
/// buffer belong to whatever params array element comes next -- so this
/// cannot NUL-terminate in place without corrupting a later argument.
/// Instead it copies the string into `Args`' small bump-allocated scratch
/// buffer (`Server::kStrScratchSize` bytes total per call, shared -- in
/// order -- by every `const char*` argument of the call) and NUL-terminates
/// there. Lifetime: the returned pointer is valid only for the duration of
/// the handler call it was decoded for; the scratch buffer is reused (and
/// the pointer invalidated) by the next dispatch.
template <>
struct ArgDecoder<const char *> {
  static bool decode(Args &args, const char *&out) {
    const char *ptr;
    size_t len;
    args.reader().read_str(ptr, len);
    if (args.reader().error()) return false;
    char *dst = args.scratch_alloc(len + 1);
    if (!dst) return false;
    memcpy(dst, ptr, len);
    dst[len] = '\0';
    out = dst;
    return true;
  }
};

// ===========================================================================
// Decode<R, ArgTs...> / Invoke<R>: apply a decoded argument list to a
// callable without std::tuple (unavailable on the fixed backend).
// ===========================================================================
//
// Each recursion level decodes one argument into a local, then recurses
// with one fewer type in ArgTs and that local appended to the trailing
// `Decoded&...` pack. Because the recursion is a normal (synchronous)
// nested call, every already-decoded local is still alive on the stack
// when the final level calls Invoke<R>::call() with the complete set --
// no tuple, no heap, just C++11 parameter packs.

template <class R>
struct Invoke {
  template <class Fn, class... Decoded>
  static void call(Fn &fn, msgpack::Writer &out, Decoded &...decoded) {
    R result = fn(decoded...);
    ValuePacker<R>::pack(out, result);
  }
};

template <>
struct Invoke<void> {
  template <class Fn, class... Decoded>
  static void call(Fn &fn, msgpack::Writer &out, Decoded &...decoded) {
    fn(decoded...);
    out.pack_nil();
  }
};

template <class R, class... ArgTs>
struct Decode;

template <class R>
struct Decode<R> {
  template <class Fn, class... Decoded>
  static bool call(Fn &fn, Args &, msgpack::Writer &out, Decoded &...decoded) {
    Invoke<R>::call(fn, out, decoded...);
    return true;
  }
};

template <class R, class Head, class... Tail>
struct Decode<R, Head, Tail...> {
  template <class Fn, class... Decoded>
  static bool call(Fn &fn, Args &args, msgpack::Writer &out, Decoded &...decoded) {
    Head value{};
    if (!ArgDecoder<Head>::decode(args, value)) return false;
    return Decode<R, Tail...>::call(fn, args, out, decoded..., value);
  }
};

// ===========================================================================
// Typed-bind thunks: adapt a free function / lambda / member function to
// the common `void(Args&, Reply&)` handler signature.
// ===========================================================================

/// Wraps any callable `Fn` (a function pointer or a lambda) with signature
/// `R(ArgTs...)`. Used by both the free-function-pointer bind() overload
/// and the lambda bind() overload -- calling `fn(decoded...)` reads the
/// same either way.
template <class Fn, class R, class... ArgTs>
struct TypedThunk {
  explicit TypedThunk(Fn f) : fn(f) {}

  void operator()(Args &args, Reply &reply) const {
    if (args.size() != sizeof...(ArgTs)) {
      reply.error("bad args");
      return;
    }
    if (!Decode<R, ArgTs...>::call(fn, args, reply.writer())) reply.error("bad args");
  }

  Fn fn;
};

/// Wraps `obj->*mfn` for `bind(name, obj, &Class::method)`. `MemberPtr` is
/// deduced by the two Server::bind() overloads below as either
/// `R (Obj::*)(ArgTs...)` or `R (Obj::*)(ArgTs...) const`; `(o->*m)(...)`
/// itself is spelled identically for both, so one template covers both.
/// An aggregate (no user-declared constructor) so `bind()` can build it
/// with `{&obj, mfn}`.
template <class Obj, class MemberPtr, class R, class... ArgTs>
struct MemberThunk {
  void operator()(Args &args, Reply &reply) const {
    if (args.size() != sizeof...(ArgTs)) {
      reply.error("bad args");
      return;
    }
    Obj *o = obj;
    MemberPtr m = mfn;
    auto invoker = [o, m](ArgTs... a) -> R { return (o->*m)(a...); };
    if (!Decode<R, ArgTs...>::call(invoker, args, reply.writer())) reply.error("bad args");
  }

  Obj *obj;
  MemberPtr mfn;
};

/// Deduces `R`/`ArgTs...` from a lambda's (or any functor's) `operator()`
/// member-function-pointer type, so `Server::bind(name, lambda)` can build
/// the right `TypedThunk` without the caller having to spell out the
/// signature. Two partial specializations cover a normal (const
/// `operator()`) lambda and a `mutable` (non-const `operator()`) one.
template <class Fn, class MemberPtr>
struct LambdaBinder; // primary: intentionally undefined (Fn::operator() always matches one specialization below)

template <class Fn, class C, class R, class... ArgTs>
struct LambdaBinder<Fn, R (C::*)(ArgTs...) const> {
  template <class HandlerFnT>
  static HandlerFnT make(Fn f) {
    return HandlerFnT(TypedThunk<Fn, R, ArgTs...>(f));
  }
};

template <class Fn, class C, class R, class... ArgTs>
struct LambdaBinder<Fn, R (C::*)(ArgTs...)> {
  template <class HandlerFnT>
  static HandlerFnT make(Fn f) {
    return HandlerFnT(TypedThunk<Fn, R, ArgTs...>(f));
  }
};

#if SERIAL_RPC_USE_STL

// ===========================================================================
// STL-only extra argument/return types (docs/PLAN.md 3b's table): decode
// from / pack as a msgpack array, layered on the same ArgDecoder /
// ValuePacker machinery used everywhere else.
// ===========================================================================

template <>
struct ArgDecoder<std::string> {
  static bool decode(Args &args, std::string &out) {
    const char *ptr;
    size_t len;
    args.reader().read_str(ptr, len);
    if (args.reader().error()) return false;
    out.assign(ptr, len);
    return true;
  }
};
template <>
struct ValuePacker<std::string> {
  static void pack(msgpack::Writer &w, const std::string &v) { w.pack_str(v.data(), v.size()); }
};

template <class T>
struct ArgDecoder<std::vector<T>> {
  static bool decode(Args &args, std::vector<T> &out) {
    size_t n;
    args.reader().read_array(n);
    if (args.reader().error()) return false;
    out.clear();
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      T value{};
      if (!ArgDecoder<T>::decode(args, value)) return false;
      out.push_back(value);
    }
    return true;
  }
};
template <class T>
struct ValuePacker<std::vector<T>> {
  static void pack(msgpack::Writer &w, const std::vector<T> &v) {
    w.pack_array(v.size());
    for (const T &e : v) ValuePacker<T>::pack(w, e);
  }
};

template <class T, size_t N>
struct ArgDecoder<std::array<T, N>> {
  static bool decode(Args &args, std::array<T, N> &out) {
    size_t n;
    args.reader().read_array(n);
    if (args.reader().error() || n != N) return false;
    for (size_t i = 0; i < N; ++i) {
      if (!ArgDecoder<T>::decode(args, out[i])) return false;
    }
    return true;
  }
};
template <class T, size_t N>
struct ValuePacker<std::array<T, N>> {
  static void pack(msgpack::Writer &w, const std::array<T, N> &v) {
    w.pack_array(N);
    for (const T &e : v) ValuePacker<T>::pack(w, e);
  }
};

template <class A, class B>
struct ArgDecoder<std::pair<A, B>> {
  static bool decode(Args &args, std::pair<A, B> &out) {
    size_t n;
    args.reader().read_array(n);
    if (args.reader().error() || n != 2) return false;
    if (!ArgDecoder<A>::decode(args, out.first)) return false;
    return ArgDecoder<B>::decode(args, out.second);
  }
};
template <class A, class B>
struct ValuePacker<std::pair<A, B>> {
  static void pack(msgpack::Writer &w, const std::pair<A, B> &v) {
    w.pack_array(2);
    ValuePacker<A>::pack(w, v.first);
    ValuePacker<B>::pack(w, v.second);
  }
};

/// Index-based recursion over a std::tuple's element types, since C++11 has
/// no fold expressions. `TupleHelper<N, N>` (I == N) is the base case.
template <size_t I, size_t N>
struct TupleHelper {
  template <class Tup>
  static bool decode(Args &args, Tup &t) {
    if (!ArgDecoder<typename std::tuple_element<I, Tup>::type>::decode(args, std::get<I>(t))) return false;
    return TupleHelper<I + 1, N>::decode(args, t);
  }
  template <class Tup>
  static void pack(msgpack::Writer &w, const Tup &t) {
    ValuePacker<typename std::tuple_element<I, Tup>::type>::pack(w, std::get<I>(t));
    TupleHelper<I + 1, N>::pack(w, t);
  }
};
template <size_t N>
struct TupleHelper<N, N> {
  template <class Tup>
  static bool decode(Args &, Tup &) {
    return true;
  }
  template <class Tup>
  static void pack(msgpack::Writer &, const Tup &) {}
};

template <class... Ts>
struct ArgDecoder<std::tuple<Ts...>> {
  static bool decode(Args &args, std::tuple<Ts...> &out) {
    size_t n;
    args.reader().read_array(n);
    if (args.reader().error() || n != sizeof...(Ts)) return false;
    return TupleHelper<0, sizeof...(Ts)>::decode(args, out);
  }
};
template <class... Ts>
struct ValuePacker<std::tuple<Ts...>> {
  static void pack(msgpack::Writer &w, const std::tuple<Ts...> &v) {
    w.pack_array(sizeof...(Ts));
    TupleHelper<0, sizeof...(Ts)>::pack(w, v);
  }
};

#endif // SERIAL_RPC_USE_STL

/// Empty class solely for computing the default InplaceFn Capacity (see
/// docs/PLAN.md 3b): `sizeof(void (CapacityProbe::*)())` is the size of a
/// member-function pointer on the target ABI, without needing a complete,
/// real class.
class CapacityProbe {};

} // namespace detail

// ===========================================================================
// Server<StreamT, MaxHandlers, BufSize, Capacity>
// ===========================================================================

/// Default InplaceFn::Capacity (docs/PLAN.md 3b): enough for a `[this]` /
/// `[&x]`-style reference capture, or an object pointer plus a
/// member-function pointer (`bind(name, obj, &Class::method)`).
enum {
  kDefaultServerCapacity = 2 * sizeof(void *) + sizeof(void (detail::CapacityProbe::*)())
};

/// \brief The device-side MsgPack-RPC dispatcher (docs/PLAN.md, components 3
///   and 3b).
///
/// `StreamT` is duck-typed, not required to inherit from anything: it only
/// needs `int available()`, `int read()`, `size_t write(const uint8_t*,
/// size_t)` and `size_t write(uint8_t)` (Arduino's `Stream`/`Print` already
/// provide exactly this; so does the `MockStream` used by the host tests).
///
/// RAM budget for the fixed backend, with the defaults (`BufSize=128`,
/// `MaxHandlers=8`) -- this is the `Server` object's own footprint, measured
/// via `avr-nm` on an actual `arduino:avr:uno` build:
///  - `_demux`: `cobs_max_encoded_size(BufSize+2)` (~131) for the incoming
///    frame + `kLineMax` (64) for the text-line buffer.
///  - `_payload_buf`: `BufSize` (128), the scratch used to build every
///    outgoing frame's payload (response or notification).
///  - `_tx_buf`: `frame_max_size(BufSize)` (~133), the final COBS+CRC+
///    delimiters bytes handed to one `stream.write()`.
///  - `kResultScratch` (64): a handler's single return value, packed
///    separately from the response header (msgid's width isn't known until
///    it's decoded) and spliced in afterwards.
///  - `kStrScratchSize` (32): NUL-terminates `const char*` arguments (see
///    `detail::ArgDecoder<const char*>`).
///  - `SERIAL_RPC_LOG_BUF_SIZE` (64, override-able): `log`'s line buffer.
///  - handler table: `MaxHandlers * (sizeof(const char*) + sizeof(InplaceFn<Capacity>))`,
///    about 12 bytes/handler with the default Capacity, per docs/PLAN.md.
/// Measured total with every default left as-is: ~720 bytes for the
/// `Server` object itself. That is *not* the whole sketch's RAM, though:
/// `HardwareSerial` adds its own ~155-byte RX/TX buffers, `Print`'s vtable
/// (shared with every other `Print`-derived object) adds ~30, and the
/// "ready" line literal (uncounted here, since it's a local, not a member)
/// adds ~22 -- enough that a sketch using every default can land just over
/// the 1 KB target on a 2 KB Uno. `examples/Blink` instantiates
/// `SerialRPC<6, 96>` instead (it only needs 5 handlers and small
/// payloads), which measures ~600 bytes for the `Server` object and ~1016
/// bytes total, comfortably under 1 KB; a sketch with more headroom (or a
/// 32-bit board) can just use the `SerialRPC<>` default.
template <class StreamT, size_t MaxHandlers = 8, size_t BufSize = 128, size_t Capacity = kDefaultServerCapacity>
class Server {
public:
  explicit Server(StreamT &stream)
      : _stream(stream),
#if !SERIAL_RPC_USE_STL
        _handler_count(0),
#endif
        _attached(false), _attach_has_timeout(false), _attach_timeout_ms(0), _last_frame_ms(0), _clock_fn(nullptr),
        _on_text(), _on_attach(), _on_detach(), _unknown_method_errors(0), _handler_error_count(0),
        _malformed_errors(0), _response_overflow_errors(0), log(this) {}

  // --- setup / main loop -----------------------------------------------

  /// Prints a plain-text "ready" line and starts detached. Call once from
  /// `setup()`.
  void begin() {
    _demux.reset();
    _attached = false;
    _attach_has_timeout = false;
    static const char kReady[] = "#serial_rpc ready v1\n";
    _stream.write(reinterpret_cast<const uint8_t *>(kReady), sizeof(kReady) - 1);
  }

  /// Non-blocking: drains everything currently `available()` on the stream
  /// through the `Demux`, dispatches any complete frame, forwards any
  /// complete text line to `on_text`, and checks the attach timeout. Call
  /// from `loop()`.
  void poll() {
    while (_stream.available() > 0) {
      const int c = _stream.read();
      if (c < 0) break;
      const DemuxEvent ev = _demux.feed(static_cast<uint8_t>(c));
      if (ev == DemuxEvent::frame) {
        handle_frame_(_demux.data(), _demux.size());
      } else if (ev == DemuxEvent::text_line) {
        if (_on_text) _on_text(_demux.line());
      }
    }
    check_attach_timeout_();
  }

  bool attached() const { return _attached; }

  // --- binding -----------------------------------------------------------

  /// Binds a free function pointer (or anything that decays to one, e.g. a
  /// capture-less lambda passed where a function-pointer parameter is
  /// expected -- though the generic overload below handles capture-less
  /// lambdas directly too).
  template <class R, class... ArgTs>
  void bind(const char *name, R (*fn)(ArgTs...)) {
    add_handler_(name, HandlerFn(detail::TypedThunk<R (*)(ArgTs...), R, ArgTs...>(fn)));
  }

  /// Binds any lambda (capture-less or capturing). `R`/`ArgTs...` are
  /// deduced from `decltype(&Fn::operator())`; generic lambdas (`auto`
  /// parameters) aren't supported, since their argument types can't be
  /// deduced this way -- write out the parameter types instead.
  ///
  /// Lifetime: anything captured by reference must outlive this `Server`.
  /// In practice that means globals, or members of the object that owns
  /// the binding (see docs/PLAN.md, component 3).
  template <class Fn>
  void bind(const char *name, Fn f) {
    add_handler_(name, detail::LambdaBinder<Fn, decltype(&Fn::operator())>::template make<HandlerFn>(f));
  }

  /// Binds a non-const member function: `rpc.bind("stop", motor, &Motor::stop);`.
  template <class Obj, class R, class... ArgTs>
  void bind(const char *name, Obj &obj, R (Obj::*mfn)(ArgTs...)) {
    using MemberPtr = R (Obj::*)(ArgTs...);
    detail::MemberThunk<Obj, MemberPtr, R, ArgTs...> thunk = {&obj, mfn};
    add_handler_(name, HandlerFn(thunk));
  }

  /// Binds a const member function.
  template <class Obj, class R, class... ArgTs>
  void bind(const char *name, Obj &obj, R (Obj::*mfn)(ArgTs...) const) {
    using MemberPtr = R (Obj::*)(ArgTs...) const;
    detail::MemberThunk<Obj, MemberPtr, R, ArgTs...> thunk = {&obj, mfn};
    add_handler_(name, HandlerFn(thunk));
  }

  /// Escape hatch for variable or complex arguments: `fn` is called with
  /// `(Args&, Reply&)` directly, with none of the argument-count checking
  /// or decoding that `bind()` generates. `args.size()` gives the
  /// parameter count; decode with `args.reader()`; set `reply.error(...)`
  /// on failure, or write a result (or nothing, for a nil result) to
  /// `reply.writer()`.
  template <class Fn>
  void bind_raw(const char *name, Fn f) {
    add_handler_(name, HandlerFn(f));
  }

  /// Map-style syntax: `rpc["set_led"] = [&](uint8_t pin, bool on){ ... };`
  /// is equivalent to `bind()`.
  class BindProxy {
  public:
    BindProxy(Server &server, const char *name) : _server(&server), _name(name) {}
    template <class Fn>
    void operator=(Fn f) { // NOLINT(*-unconventional-assign-operator)
      _server->bind(_name, f);
    }

  private:
    Server *_server;
    const char *_name;
  };

  BindProxy operator[](const char *name) { return BindProxy(*this, name); }

  // --- outbound: notifications & logging ---------------------------------

  /// Sends `[2, name, [args...]]` only while attached (per docs/PLAN.md's
  /// attach/detach design: an unsolicited frame while detached would show
  /// up as garbage in a plain Serial Monitor). Returns false and sends
  /// nothing otherwise, or if the payload doesn't fit `BufSize`.
  template <class... ArgTs>
  bool notify(const char *name, const ArgTs &...args) {
    if (!_attached) return false;
    msgpack::Writer w(_payload_buf, sizeof(_payload_buf));
    w.pack_array(3);
    w.pack(2);
    w.pack(name);
    w.pack_array(sizeof...(ArgTs));
    pack_notify_args_(w, args...);
    if (w.overflow()) {
      ++_response_overflow_errors;
      return false;
    }
    send_frame_(_payload_buf, w.size());
    return true;
  }

#if SERIAL_RPC_USE_STL
  using TextCb = std::function<void(const char *)>;
  using VoidCb = std::function<void()>;
#else
  using TextCb = void (*)(const char *);
  using VoidCb = void (*)();
#endif

  /// Called from `poll()` for every text line that isn't part of a frame
  /// (typed by a human in a terminal, or printed by other code sharing the
  /// stream).
  void on_text(TextCb cb) { _on_text = cb; }
  /// Called once when `rpc.attach` succeeds.
  void on_attach(VoidCb cb) { _on_attach = cb; }
  /// Called once when the device becomes detached, whether by `rpc.detach`
  /// or by an attach timeout.
  void on_detach(VoidCb cb) { _on_detach = cb; }

  // --- diagnostics ---------------------------------------------------------

  size_t handler_count() const { return handler_count_(); }
  uint32_t unknown_method_errors() const { return _unknown_method_errors; }
  uint32_t handler_errors() const { return _handler_error_count; }
  uint32_t malformed_errors() const { return _malformed_errors; }
  uint32_t response_overflow_errors() const { return _response_overflow_errors; }
  uint32_t crc_errors() const { return _demux.crc_errors(); }
  uint32_t cobs_errors() const { return _demux.cobs_errors(); }
  uint32_t frame_overflow_errors() const { return _demux.overflow_errors(); }

  /// Injects a millis()-like clock for the attach timeout. On Arduino this
  /// defaults to `millis()`; off Arduino (host tests) it must be set
  /// explicitly for `attach(timeout_ms)` to ever expire.
  void set_clock(uint32_t (*fn)()) { _clock_fn = fn; }

  // --- logging -------------------------------------------------------------

  /// A `Print` subclass on Arduino (so `rpc.log.println(x)` etc. work
  /// exactly like `Serial.println`), or a small standalone class with the
  /// same handful of methods off Arduino (for host tests). Buffers up to a
  /// newline (`SERIAL_RPC_LOG_BUF_SIZE` bytes, default 64), then emits the
  /// whole line at once: as plain text while detached, or as a framed
  /// `[2, "log", [level, line]]` notification while attached.
  class Log
#if defined(ARDUINO)
      : public Print
#endif
  {
  public:
    explicit Log(Server *owner) : _owner(owner), _len(0), _level(2) {}

    /// Sets the level tag used by subsequent lines (default 2 = info).
    void level(uint8_t lvl) { _level = lvl; }

#if defined(ARDUINO)
    size_t write(uint8_t b) override {
      handle_byte_(b);
      return 1;
    }
    using Print::write;
#else
    size_t write(uint8_t b) {
      handle_byte_(b);
      return 1;
    }
    size_t write(const uint8_t *buf, size_t n) {
      for (size_t i = 0; i < n; ++i) write(buf[i]);
      return n;
    }
    size_t print(const char *s) {
      const size_t n = strlen(s);
      write(reinterpret_cast<const uint8_t *>(s), n);
      return n;
    }
    size_t println(const char *s) {
      const size_t n = print(s);
      write(static_cast<uint8_t>('\n'));
      return n + 1;
    }
    size_t println() {
      write(static_cast<uint8_t>('\n'));
      return 1;
    }
#endif

  private:
    void handle_byte_(uint8_t b) {
      if (b == '\r') return; // swallowed; a following '\n' (Print::println's usual "\r\n") ends the line
      if (b == '\n') {
        flush_();
        return;
      }
      if (_len + 1 >= SERIAL_RPC_LOG_BUF_SIZE) flush_(); // forced line break: buffer is full
      _buf[_len++] = static_cast<char>(b);
    }
    void flush_() {
      _buf[_len] = '\0';
      _owner->emit_log_line_(_level, _buf, _len);
      _len = 0;
    }

    Server *_owner;
    char _buf[SERIAL_RPC_LOG_BUF_SIZE];
    size_t _len;
    uint8_t _level;
  };

  /// `rpc.log_at(level).println(...)`: sets the level for the next line(s)
  /// and returns `log` for chaining. Equivalent to `log.level(level)`.
  Log &log_at(uint8_t level) {
    log.level(level);
    return log;
  }

  static const size_t kStrScratchSize = 32;
  static const size_t kResultScratch = 64;
  static const size_t kLineMax = 64;
  static const size_t kFrameMax = cobs_max_encoded_size(BufSize + 2);

private:
  friend class Log;

#if SERIAL_RPC_USE_STL
  using HandlerFn = std::function<void(Args &, Reply &)>;
  struct Entry {
    std::string name;
    HandlerFn fn;
  };
#else
  using HandlerFn = detail::InplaceFn<Capacity>;
  struct Entry {
    const char *name = nullptr;
    HandlerFn fn;
  };
#endif

  // --- dispatch ------------------------------------------------------------

  void handle_frame_(const uint8_t *data, size_t len) {
    // Any frame that passed COBS+CRC validation counts as "the host is
    // alive" for the attach timeout, whether or not its contents go on to
    // parse as a well-formed RPC message.
    _last_frame_ms = now_ms_();

    // Every local below is zero-initialized at its declaration, even though
    // each one is only read after the Reader:: call that fills it is
    // checked for error(): avr-gcc's -Os flow analysis can't always see
    // that a Reader method (called through a template/inline chain) always
    // writes its output parameter when error() comes back false, and warns
    // -Wmaybe-uninitialized without the explicit initializer.
    msgpack::Reader r(data, len);
    size_t arr_n = 0;
    r.read_array(arr_n);
    if (r.error()) {
      ++_malformed_errors;
      return;
    }
    int msg_type = 0;
    r.read(msg_type);
    if (r.error()) {
      ++_malformed_errors;
      return;
    }

    if (msg_type == 0) { // request: [0, msgid, method, params]
      if (arr_n != 4) {
        ++_malformed_errors;
        return;
      }
      uint32_t msgid = 0;
      r.read(msgid);
      const char *method_ptr = nullptr;
      size_t method_len = 0;
      r.read_str(method_ptr, method_len);
      size_t params_n = 0;
      r.read_array(params_n);
      if (r.error()) {
        ++_malformed_errors;
        return;
      }
      Args args(r, params_n, _str_scratch, kStrScratchSize);
      uint8_t result_buf[kResultScratch];
      msgpack::Writer result_w(result_buf, sizeof(result_buf));
      Reply reply(result_w);
      dispatch_(method_ptr, method_len, args, reply);
      send_response_(msgid, reply);
    } else if (msg_type == 2) { // notification: [2, method, params]
      if (arr_n != 3) {
        ++_malformed_errors;
        return;
      }
      const char *method_ptr = nullptr;
      size_t method_len = 0;
      r.read_str(method_ptr, method_len);
      size_t params_n = 0;
      r.read_array(params_n);
      if (r.error()) {
        ++_malformed_errors;
        return;
      }
      Args args(r, params_n, _str_scratch, kStrScratchSize);
      uint8_t result_buf[kResultScratch];
      msgpack::Writer result_w(result_buf, sizeof(result_buf));
      Reply reply(result_w); // result/error discarded: notifications get no response
      dispatch_(method_ptr, method_len, args, reply);
    } else {
      ++_malformed_errors; // e.g. a stray response (type 1): this device never sends requests
    }
  }

  void dispatch_(const char *ptr, size_t len, Args &args, Reply &reply) {
    if (dispatch_builtin_(ptr, len, args, reply)) {
      if (reply.failed()) ++_handler_error_count;
      return;
    }
    if (dispatch_user_(ptr, len, args, reply)) {
      if (reply.failed()) ++_handler_error_count;
      return;
    }
    reply.error("unknown method");
    ++_unknown_method_errors;
  }

  static bool method_is_(const char *ptr, size_t len, const char *lit) {
    const size_t lit_len = strlen(lit);
    return len == lit_len && memcmp(ptr, lit, lit_len) == 0;
  }

  /// Built-in methods (docs/PLAN.md's "Wire protocol" section). These are
  /// dispatched before the user handler table and don't consume a
  /// `MaxHandlers` slot. `rpc.list` deliberately excludes them: it enumerates
  /// what the *sketch* bound, since the four built-ins are always assumed
  /// present by any host client speaking this protocol.
  bool dispatch_builtin_(const char *ptr, size_t len, Args &args, Reply &reply) {
    if (method_is_(ptr, len, "rpc.ping")) {
      reply.writer().pack(static_cast<int>(kProtocolVersion));
      return true;
    }
    if (method_is_(ptr, len, "rpc.list")) {
      reply.writer().pack_array(handler_count_());
      pack_method_names_(reply.writer());
      return true;
    }
    if (method_is_(ptr, len, "rpc.attach")) {
      uint32_t timeout_ms = 0;
      if (args.size() >= 1) {
        args.reader().read(timeout_ms);
        if (args.reader().error()) {
          reply.error("bad args");
          return true;
        }
      }
      _attached = true;
      _attach_has_timeout = timeout_ms > 0;
      _attach_timeout_ms = timeout_ms;
      _last_frame_ms = now_ms_();
      if (_on_attach) _on_attach();
      reply.writer().pack_array(2);
      reply.writer().pack(static_cast<int>(kProtocolVersion));
      reply.writer().pack(static_cast<uint32_t>(handler_count_()));
      return true;
    }
    if (method_is_(ptr, len, "rpc.detach")) {
      const bool was_attached = _attached;
      _attached = false;
      _attach_has_timeout = false;
      if (was_attached && _on_detach) _on_detach();
      return true;
    }
    return false;
  }

#if SERIAL_RPC_USE_STL
  void pack_method_names_(msgpack::Writer &w) {
    for (const Entry &e : _handlers) w.pack_str(e.name.data(), e.name.size());
  }
  bool dispatch_user_(const char *ptr, size_t len, Args &args, Reply &reply) {
    for (Entry &e : _handlers) {
      if (e.name.size() == len && memcmp(e.name.data(), ptr, len) == 0) {
        e.fn(args, reply);
        return true;
      }
    }
    return false;
  }
  void add_handler_(const char *name, HandlerFn fn) {
    const std::string name_str(name);
    for (Entry &e : _handlers) {
      if (e.name == name_str) {
        e.fn = fn;
        return;
      }
    }
    Entry e;
    e.name = name_str;
    e.fn = fn;
    _handlers.push_back(e);
  }
  size_t handler_count_() const { return _handlers.size(); }
#else
  void pack_method_names_(msgpack::Writer &w) {
    for (size_t i = 0; i < _handler_count; ++i) w.pack(_handlers[i].name);
  }
  bool dispatch_user_(const char *ptr, size_t len, Args &args, Reply &reply) {
    for (size_t i = 0; i < _handler_count; ++i) {
      if (method_is_(ptr, len, _handlers[i].name)) {
        _handlers[i].fn(args, reply);
        return true;
      }
    }
    return false;
  }
  void add_handler_(const char *name, HandlerFn fn) {
    for (size_t i = 0; i < _handler_count; ++i) {
      if (method_is_(name, strlen(name), _handlers[i].name)) {
        _handlers[i].fn = fn;
        return;
      }
    }
    if (_handler_count < MaxHandlers) {
      _handlers[_handler_count].name = name;
      _handlers[_handler_count].fn = fn;
      ++_handler_count;
    } else {
      ++_response_overflow_errors; // handler table full; documented as a silent drop plus this counter
    }
  }
  size_t handler_count_() const { return _handler_count; }
#endif

  // --- response / notification framing --------------------------------

  void pack_notify_args_(msgpack::Writer &) {}
  template <class First, class... Rest>
  void pack_notify_args_(msgpack::Writer &w, const First &first, const Rest &...rest) {
    detail::ValuePacker<First>::pack(w, first);
    pack_notify_args_(w, rest...);
  }

  /// Builds `[1, msgid, error|nil, result|nil]` into `_payload_buf` and
  /// sends it.
  ///
  /// On failure, the whole array (including the, possibly long, user- or
  /// built-in-supplied error string) is packed directly into `_payload_buf`
  /// in one pass. On success, the header (array/type/msgid/nil-error-slot)
  /// is small and *bounded* (at most 8 bytes: a fixarray byte, a fixint
  /// type byte, up to 5 bytes for a uint32 msgid, one nil byte), so it's
  /// built first with a tiny local Writer; the handler's already-packed
  /// result bytes (from `reply.writer()`, a *separate* small Writer -- see
  /// the class comment on `Reply`) are then copied in raw, since msgpack
  /// array elements are just concatenated encodings -- no need to decode
  /// and re-pack the result to splice it into place.
  void send_response_(uint32_t msgid, Reply &reply) {
    bool overflowed = false;
    size_t total = 0;

    if (reply.failed()) {
      msgpack::Writer w(_payload_buf, sizeof(_payload_buf));
      w.pack_array(4);
      w.pack(1);
      w.pack(msgid);
      w.pack(reply.error_message());
      w.pack_nil();
      overflowed = w.overflow();
      total = w.size();
    } else {
      uint8_t hdr_buf[8];
      msgpack::Writer hw(hdr_buf, sizeof(hdr_buf));
      hw.pack_array(4);
      hw.pack(1);
      hw.pack(msgid);
      hw.pack_nil(); // error slot
      overflowed = hw.overflow() || reply.writer().overflow();
      if (!overflowed) {
        memcpy(_payload_buf, hw.data(), hw.size());
        total = hw.size();
        const size_t result_len = reply.writer().size();
        if (result_len == 0) {
          msgpack::Writer nil_w(_payload_buf + total, sizeof(_payload_buf) - total);
          nil_w.pack_nil();
          if (nil_w.overflow()) {
            overflowed = true;
          } else {
            total += nil_w.size();
          }
        } else if (total + result_len <= sizeof(_payload_buf)) {
          memcpy(_payload_buf + total, reply.writer().data(), result_len);
          total += result_len;
        } else {
          overflowed = true;
        }
      }
    }

    if (overflowed) {
      ++_response_overflow_errors;
      msgpack::Writer ew(_payload_buf, sizeof(_payload_buf));
      ew.pack_array(4);
      ew.pack(1);
      ew.pack(msgid);
      ew.pack("response too large");
      ew.pack_nil();
      total = ew.size();
    }

    send_frame_(_payload_buf, total);
  }

  void send_frame_(const uint8_t *payload, size_t len) {
    const size_t n = frame_encode(payload, len, _tx_buf, sizeof(_tx_buf));
    if (n == 0) {
      ++_response_overflow_errors;
      return;
    }
    _stream.write(_tx_buf, n);
  }

  void emit_log_line_(uint8_t level, const char *line, size_t len) {
    if (_attached) {
      notify("log", static_cast<int>(level), line);
    } else {
      _stream.write(reinterpret_cast<const uint8_t *>(line), len);
      _stream.write(static_cast<uint8_t>('\n'));
    }
  }

  // --- attach timeout / clock -------------------------------------------

  void check_attach_timeout_() {
    if (_attached && _attach_has_timeout) {
      const uint32_t now = now_ms_();
      if (now - _last_frame_ms >= _attach_timeout_ms) {
        _attached = false;
        _attach_has_timeout = false;
        if (_on_detach) _on_detach();
      }
    }
  }

#if defined(ARDUINO)
  static uint32_t default_clock_() { return static_cast<uint32_t>(millis()); }
#else
  static uint32_t default_clock_() { return 0; }
#endif
  uint32_t now_ms_() const { return _clock_fn ? _clock_fn() : default_clock_(); }

  // --- members -------------------------------------------------------------

  StreamT &_stream;
  Demux<kFrameMax, kLineMax> _demux;
  uint8_t _payload_buf[BufSize];
  uint8_t _tx_buf[frame_max_size(BufSize)];
  char _str_scratch[kStrScratchSize];

#if SERIAL_RPC_USE_STL
  std::vector<Entry> _handlers;
#else
  Entry _handlers[MaxHandlers];
  size_t _handler_count;
#endif

  bool _attached;
  bool _attach_has_timeout;
  uint32_t _attach_timeout_ms;
  uint32_t _last_frame_ms;
  uint32_t (*_clock_fn)();

  TextCb _on_text;
  VoidCb _on_attach;
  VoidCb _on_detach;

  uint32_t _unknown_method_errors;
  uint32_t _handler_error_count;
  uint32_t _malformed_errors;
  uint32_t _response_overflow_errors;

public:
  // Declared last so it initializes after every buffer/state member above
  // (member init order follows declaration order, not the constructor
  // initializer-list's order); Log's constructor only stores `this`, so
  // that's all it needs at construction time.
  Log log;
};

#if defined(ARDUINO)
/// Convenience alias for sketches: `SerialRPC<> rpc(Serial);`. Templated on
/// Arduino's `Stream` base class (virtual `available`/`read`/`write`)
/// rather than on the exact per-core serial type (`HardwareSerial`,
/// `Serial_`, `UART`, `USBCDC`, ...), so the same alias works unchanged on
/// every core.
template <size_t MaxHandlers = 8, size_t BufSize = 128>
using SerialRPC = Server<Stream, MaxHandlers, BufSize>;
#endif

} // namespace serial_rpc

#undef SERIAL_RPC_IS_TRIVIALLY_DESTRUCTIBLE_
#undef SERIAL_RPC_IS_TRIVIALLY_COPYABLE_
