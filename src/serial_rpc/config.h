// SPDX-License-Identifier: Apache-2.0
/// \file config.h
/// \brief Compile-time configuration for the device-side RPC server: which
///   storage backend it uses (`server.h`, component 3b) and a couple of
///   small tunables.
///
/// Must compile as C++11 with avr-gcc 7.3 (freestanding headers only) as
/// well as warning-free as C++20 with Clang on the host, since every build
/// of the fixed backend -- including the one used by the host unit tests --
/// includes this header.
#pragma once

/// \def SERIAL_RPC_USE_STL
/// Selects `server.h`'s storage backend (see docs/PLAN.md, component 3b):
///  - `0`: the fixed backend. A bounded array of handlers, `InplaceFn`
///    (fixed-capacity, no heap) callables, `const char*` method names. This
///    is what avr-gcc / AVR cores get, and what any core can force.
///  - `1`: the STL backend. `std::vector`-backed handler storage,
///    `std::function` callables, `std::string` names, and extra typed
///    argument/return support for `std::string`, `std::vector<T>`,
///    `std::array<T,N>` and `std::pair`/`std::tuple`.
///
/// If the user (or a build system) already `#define`s `SERIAL_RPC_USE_STL`
/// before this header is first included, that value is used as-is and the
/// auto-detection below is skipped entirely -- this is the escape hatch for
/// a 32-bit part that wants to save flash by forcing the fixed backend, or
/// an AVR board with a add-on STL (e.g. ArduinoSTL) that wants to force the
/// STL backend.
///
/// Auto-detection, per docs/PLAN.md 3b, in order:
///  1. `__AVR__` / `ARDUINO_ARCH_AVR` / `ARDUINO_ARCH_MEGAAVR` defined ->
///     force `0` (avr-gcc has no `<functional>`/`<vector>` at all).
///  2. Otherwise, if the compiler doesn't even have `__has_include` (very
///     old toolchains), `0` is the conservative default.
///  3. Otherwise, `1` iff both `<functional>` and `<vector>` are available.
#ifndef SERIAL_RPC_USE_STL

#if defined(__AVR__) || defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
#define SERIAL_RPC_USE_STL 0
#elif defined(__has_include)
#if __has_include(<functional>) && __has_include(<vector>)
#define SERIAL_RPC_USE_STL 1
#else
#define SERIAL_RPC_USE_STL 0
#endif
#else
#define SERIAL_RPC_USE_STL 0
#endif

#endif // SERIAL_RPC_USE_STL

/// \def SERIAL_RPC_LOG_BUF_SIZE
/// Line-buffering capacity, in bytes, of `Server::log` (component 3's
/// `Print`-like logger): bytes accumulate here until a `\n` is seen (or the
/// buffer fills, which forces an early, truncated flush), then the whole
/// line is emitted in one shot -- as a plain text line while detached, or
/// as one framed `"log"` notification while attached. This is what
/// docs/PLAN.md means by "buffer size configurable, default 64": it is not
/// a `Server` template parameter (the four template parameters are fixed by
/// the public API), but a build-wide override, `#define`d before this
/// header is first included, exactly like `SERIAL_RPC_USE_STL` above.
#ifndef SERIAL_RPC_LOG_BUF_SIZE
#define SERIAL_RPC_LOG_BUF_SIZE 64
#endif

namespace serial_rpc {

/// Wire protocol version, returned by the `rpc.ping` built-in and as the
/// first element of `rpc.attach`'s reply. Bump this if the framing or the
/// message-shape rules in docs/PLAN.md's "Wire protocol" section ever
/// change in a way a host client needs to detect.
enum { kProtocolVersion = 1 };

} // namespace serial_rpc
