# Plan: Arduino ⇄ host MsgPack-RPC over serial

## Context
`TASK.md` asks for a small framework that makes it easy to send commands with typed arguments from a host PC to an Arduino over serial, using MsgPack in an RPC style. The Arduino side should feel like Arduino code (register callbacks, call `rpc.poll()` in `loop()`). The host side should be a header-only `RPC` class built on the existing `SerialPort` ([src/serialport.hpp](src/serialport.hpp) + [src/serialport.cpp](src/serialport.cpp), ~1700 lines, C++20, POSIX/Win32), which also has to be merged into a single header.

Decisions already made:
- **Every board, AVR included.** The baseline device code is C++11 with no STL, so it builds with avr-gcc 7.3. On 32-bit boards an STL backend is switched on automatically (section 3b).
- **Our own tiny MsgPack codec**, one dependency-free header shared by both sides.
- **Plain text and RPC frames share the line, with an attach/detach toggle** (revised; this replaces "everything framed"). Details are in the next section.
- **Arduino builds with arduino-cli.** The host builds with CMake + Ninja.

## Wire protocol
- **Messages** follow the standard [MessagePack-RPC](https://github.com/msgpack-rpc/msgpack-rpc/blob/master/spec.md) shapes:
  - Request `[0, msgid, "method", [args...]]`
  - Response `[1, msgid, error|nil, result|nil]`
  - Notification `[2, "method", [args...]]`, used in both directions.
- **Frames can be told apart from text, so no mode switch is needed:**
  - A frame is `0x00 · COBS(payload ‖ CRC16-CCITT) · 0x00`.
  - COBS output never contains `0x00`, and printable text never contains it either. So a byte stream can mix `Serial.print()` text and RPC frames freely, and a `Demux` state machine separates them.
  - A frame is sent with a single `write()` of a prepared buffer, so text can't be spliced into it. The one exception is a print from an ISR, which is documented as unsupported.
- **The `Demux` recovers from errors on its own.** It is shared code and runs on both sides:
  - In TEXT state, bytes build up into lines, and `\n` emits a text line. A `0x00` switches to FRAME.
  - In FRAME state, bytes build up until the next `0x00`:
    - An empty frame (`00 00`) stays in FRAME. This repairs a stream after joining in the middle.
    - COBS and CRC pass: dispatch the frame and go back to TEXT.
    - COBS or CRC fail: hand the collected bytes back as text, count an error, and treat the `0x00` just read as the start of a new frame.
    - The frame grows past `max_frame`: hand it back as text and go to TEXT.
  - As a result, attaching to a device that is already streaming text resyncs within one frame at most.
- **Your proposed toggle, recast as `attach`/`detach`.** It sets whether the device *starts* framed traffic. It does not decide what the wire is able to carry.
  - **Detached** (the default after reset): the device only sends frames in reply to requests. `rpc.log` prints plain text lines. The Arduino Serial Monitor looks exactly like a normal sketch.
  - `rpc.attach` is a request that replies with protocol version and method count. After it the device:
    - sends unsolicited frames (`notify()` events)
    - sends `rpc.log` output as framed `"log"` notifications, with a level and atomic lines
  - `rpc.detach` returns to plain text. A **notification** is used here, so no reply is expected.
  - If the host crashes, the device stays attached until it is reset or gets a `detach`. That is harmless, because the host's `Demux` still shows everything. An optional `attach(timeout_ms)` makes the device detach itself if no frame arrives for that long.
  - Plain `Serial.print()` calls, from the sketch or from third-party libraries, are always safe in either state. They reach the host as text lines.
  - I recommend this over a pure framed/unframed toggle. A pure toggle holds state on both ends, and that state goes out of sync on a device reset or host crash. It would also break any stray `Serial.print()` while framed. The toggle is still useful for cosmetics (Serial Monitor output) and structure (framed logs), and that is the job it does here.
- **Text sent host → device** goes the same way. Bytes that are not frames are passed to an optional device callback, `rpc.on_text([](const char* line){...})`, so a sketch can keep a human-typed command interface next to RPC.
- **Built-in methods:** `rpc.ping` (returns the version), `rpc.list` (returns method names), `rpc.attach`, `rpc.detach`.
- **Boot:** with DTR auto-reset, the host waits for a `"ready"` line or pings until it gets a reply, then attaches.
- **Errors:** strings in the response's `error` slot, e.g. `"unknown method"` or `"bad args"`, or anything the handler sets.
- **Rule for users, documented:** don't `Serial.write()` raw binary containing `0x00` on the same port.

## Layout
```
arduino/SerialRPC/                 # Arduino library (arduino-cli installable via --library)
  library.properties
  src/SerialRPC.h                  # umbrella include
  src/serial_rpc/msgpack_lite.h    # SHARED codec: Writer (to buffer) + Reader (cursor over buffer), C++11
  src/serial_rpc/framing.h         # SHARED COBS encode/decode + CRC16, incremental frame receiver
  src/serial_rpc/server.h          # device dispatcher, templated on Stream-like type
  examples/Blink/Blink.ino         # set_led, set_period, get_status, logging
host/include/
  serialport.hpp                   # merged header-only SerialPort (from src/)
  serial_rpc.hpp                   # header-only RPC client; includes the shared headers above
host/examples/rpc_cli.cpp          # cxxopts + fmt: `rpc_cli -p /dev/cu.usbmodem1 set_led 13 true`
tests/                             # doctest via FetchContent
CMakeLists.txt                     # host lib (INTERFACE), examples, tests; include path to arduino/SerialRPC/src
```
There is a single source of truth for the codec and the framing: the host includes them straight from the Arduino library folder.

## Components

### 1. `msgpack_lite.h` (shared, C++11, no heap, no STL)
- **`Writer`** writes into a caller-provided `uint8_t*` buffer and sets an overflow flag instead of throwing. It supports nil, bool, int/uint (smallest encoding), float32/64, str, bin, array/map headers, and a variadic `pack(args...)`.
- **`Reader`** is a cursor over a buffer. It provides `type()`, `read(int32_t&)` and the other scalar overloads, `read_str(const char*&, size_t&)` (zero-copy), `array_size()`, `skip()`, and a sticky error flag.
- **Numeric reads convert when the value fits.** For example, a host that sends a positive fixint can land in `uint8_t`, `int16_t`, `long` or `float`. This matters because the host doesn't know the device's integer widths.
- On AVR `double` is 32-bit, so float64 values are narrowed with care.

### 2. `framing.h` (shared)
- `cobs_encode` / `cobs_decode`, and `crc16_ccitt`.
- `frame_encode(payload, out)` produces `00 · COBS · 00`.
- `Demux<FrameMax, LineMax>` takes one byte at a time with `feed(uint8_t)` and returns `none`, `text_line` or `frame`. The line or payload is then available through `data()` and `size()`. Error counters are exposed.
  - It implements the recovery rules above, with no heap.
  - On AVR, line capture can be switched off (`LineMax = 0`), which drops the text bytes.

### 3. Device `serial_rpc::Server<Stream, MaxHandlers=8, BufSize=128>` in `server.h`
- **Fixed handler table:** each entry is `{const char* name; InplaceFn fn}`. No heap is used.
- **`InplaceFn<Capacity>`** is a small type-erased callable that stores the callable object inside the handler slot, much like `std::function` but with a fixed buffer and no heap.
  - The buffer holds `Capacity` bytes, default `2*sizeof(void*) + sizeof(void(Dummy::*)())`. That is enough for a couple of reference captures, `[this]`, or an object pointer plus a member-function pointer.
  - A `static_assert` fires on oversized captures, with a clear message telling the user to raise `Capacity` or capture by reference.
  - Placement new is used, with a local declaration because AVR has no `<new>`.
  - Callables must be trivially destructible (checked with `static_assert`), since handlers are never removed.
- **Typed binding (the main expressive feature).** State outside the lambda is reachable without globals:
  ```cpp
  SerialRPC<> rpc(Serial);
  Motor motor(9);
  void set_led(uint8_t pin, bool on) { digitalWrite(pin, on); }
  rpc.bind("set_led", set_led);                                  // free function
  rpc.bind("set_period", [](uint32_t ms) { period = ms; });      // capture-less (globals)
  rpc.bind("speed", [&motor](int s) { motor.set_speed(s); });    // capturing lambda
  rpc.bind("stop", motor, &Motor::stop);                         // object + member fn
  // inside a class: rpc.bind("reset", [this]() { reset(); });
  ```
  - A C++11 variadic template deduces `R(Args...)` from function pointers, member-function pointers, and lambdas (via `decltype(&F::operator())`). It generates a thunk that checks the argument count, decodes each argument with `Reader`, invokes the callable, and packs the return value (`void` becomes `nil`).
  - Generic lambdas (`auto` parameters) are not supported, because their argument types cannot be deduced.
  - Lifetime caveat, documented in the header: anything captured by reference must outlive `rpc`. In practice the captured objects are globals, or members of the object that owns the binding.
- **Raw escape hatch** for variable or complex arguments: `rpc.bind_raw("cfg", [](serial_rpc::Args& a, serial_rpc::Reply& r){ ... })`. `r.error("...")` sets an error.
- **Runtime:**
  - `begin()` prints a `"ready"` text line. The device starts detached.
  - `poll()` is called from `loop()`. It is non-blocking: it drains `stream.available()` into the `Demux`, dispatches frames, forwards text lines to `on_text`, and checks the attach timeout.
  - `notify("event", args...)` is sent only while attached. It returns false otherwise.
  - `rpc.log` is a `Print` subclass, so `rpc.log.println(x)` and `rpc.log.printf(...)` work where the core has them. It buffers one line and then either writes plain text (detached) or sends a framed `"log"` notification (attached).
  - `attached()` returns the current state. `on_attach` and `on_detach` hooks are optional.
- **Templated on the stream type,** so the same class runs in host unit tests against a mock stream.

### 3b. Optional STL backend for 32-bit boards (in `server.h` + `config.h`)
- **Selection lives in `src/serial_rpc/config.h`.**
  - `SERIAL_RPC_USE_STL` is set to 1 when all of these hold:
    - not `__AVR__` / `ARDUINO_ARCH_AVR` / `ARDUINO_ARCH_MEGAAVR`
    - `__has_include(<functional>)`
    - `__has_include(<vector>)`
  - Otherwise it is set to 0.
  - The user can force either way with `#define SERIAL_RPC_USE_STL 0/1` before the include. This helps small 32-bit parts that want to save flash, or AVR with ArduinoSTL.
  - The STL cores in practice are ESP32/ESP8266, RP2040 (both cores), SAMD, Renesas (UNO R4), Teensy and STM32.
- **One public API, two storage policies:**

  | | Fixed (AVR / forced) | STL |
  |---|---|---|
  | Handler storage | array of `MaxHandlers` | `std::vector<Entry>` (unbounded; linear search, which is faster than a map for a handful of names) |
  | Callable | `InplaceFn<Capacity>`, trivially destructible | `std::function<void(Args&, Reply&)>`: any capture size, owning captures (`[s = std::string(...)]`), non-trivial destructors |
  | Method name | `const char*` (must be a literal or static) | `std::string` (may be built at runtime, e.g. `"motor" + String(i)`) |
  | Extra typed args | scalars, `const char*` / `(ptr,len)` strings | also `std::string`, `std::vector<T>`, `std::array<T,N>`, `std::pair`/`std::tuple` from arrays; return values of the same types |
  | Buffers | template `BufSize` | the same `BufSize` default, which can grow up to a configurable max |

- **Map-style syntax, as sketched in `TASK.md`, works in both modes** through a proxy:
  - `rpc["set_led"] = [&](uint8_t pin, bool on){ ... };` is equivalent to `bind()`.
  - In fixed mode the key must be a string literal, and assigning a lambda that is too large triggers the same `static_assert`.
- **Sketches written against the common API compile unchanged on every board.** STL-only extras (owning captures, `std::string` args) are opt-in, and on AVR they fail with a readable `static_assert` rather than a template error cascade.
- **No exceptions on either path,** because many cores build with `-fno-exceptions`. Errors travel back to the host in the response `error` slot.
- **The implementation pattern keeps the `#if` blocks few and local.** A `detail::HandlerTable` has two definitions selected by `SERIAL_RPC_USE_STL`, and a `detail::Callable` alias switches between `InplaceFn` and `std::function`. The typed-thunk generator and the codec are shared. Extra `Reader`/`Writer` overloads for STL types sit inside `#if SERIAL_RPC_USE_STL`.
- **AVR RAM budget:** roughly `2*BufSize + MaxHandlers*(2 + 2 + Capacity)` bytes, which comes to about 12 bytes per handler with the default capacity. Method names are `const char*`. Storing them with `F()` / PROGMEM is a possible later optimisation.

### 4. Host `serialport.hpp` (header-only merge)
- Move all definitions from `serialport.cpp` into the header as `inline` members and functions, and put file-local helpers in a `detail` namespace.
- The Win32 path includes `<windows.h>` under `WIN32_LEAN_AND_MEAN` and `NOMINMAX`. The `NativeHandle = void*` trick can stay, or the header can now use `HANDLE` directly.
- `struct State` stays defined per platform inside the header.
- Delete `src/` once the merged header compiles and passes the smoke test.

### 5. Host `serial_rpc.hpp` (C++20, header-only)
- **`Value`**, a small owning dynamic type: `std::variant<nil, bool, int64_t, uint64_t, double, std::string, bin, std::vector<Value>, map>`.
  - It is decoded from `msgpack_lite::Reader`.
  - Accessors are `as<T>()`.
  - `operator<<` / `fmt::formatter` print it.
  - It replaces `std::vector<msgpack::object>` from `TASK.md`.
- **`RPC`:**
  ```cpp
  RPC rpc(port, {.timeout = 1s});
  rpc.connect();                             // wait ready/ping, then rpc.attach
  Value v = rpc.call("get_temp");            // generic
  int t   = rpc.call<int>("get_temp");       // typed
  rpc.call("set_led", 13, true);             // variadic args packed via Writer
  rpc.notify("set_period", 250);             // fire-and-forget
  rpc.on_log([](int level, std::string_view s){ ... });  // framed logs
  rpc.on_text([](std::string_view line){ ... });         // raw Serial.print lines
  rpc.on("button", [](const Value& args){ ... });
  rpc.send_text("help");                     // raw line to device's on_text
  rpc.disconnect();                          // rpc.detach notification; also in dtor
  ```
- **Behaviour:**
  - `call()` sends a request and then reads through the `Demux` until the matching `msgid` arrives, dispatching logs, notifications and text lines as they come in.
  - By default `on_text` prints to stderr.
  - `connect()` is optional. Calls work while detached too, which covers the "one-off call from the terminal" case.
  - It throws `RemoteError` on an error response and `TimeoutError` on a timeout.
  - `poll()` pumps incoming notifications when no call is in flight.
  - It is single-threaded, like `SerialPort`.

### 6. Build
- **CMake** (`cmake -Bbuild -G Ninja && cmake --build build`), with FetchContent for `fmt`, `cxxopts` and `doctest`. Targets:
  - `serial_rpc` (INTERFACE)
  - `rpc_cli`
  - `serial_rpc_tests`
- **Arduino:** `arduino-cli compile -b arduino:avr:uno --library arduino/SerialRPC arduino/SerialRPC/examples/Blink`. Also compile once for a 32-bit core, e.g. `arduino:samd:mkr1000` or `esp32:esp32:esp32`, if that core is installed.

## Implementation order
1. `msgpack_lite.h` + tests (round-trip every type, numeric coercion, overflow and truncated input).
2. `framing.h` + tests. Cover:
   - COBS vectors, including 0x00 runs and 254-byte blocks
   - CRC
   - the `Demux` on text and frames interleaved
   - joining in the middle of a frame
   - a corrupted frame falling back to text
   - oversized frames
   - partial text lines split by a frame
3. `server.h` + tests using a loopback mock stream. Build the test suite twice, once with `SERIAL_RPC_USE_STL=0` and once with `=1`, so both backends are covered on the host. Cover:
   - bind and dispatch for free functions, capturing lambdas, and member functions
   - argument-count and type errors
   - unknown methods
   - raw handlers
   - `rpc.list`
   - attach, detach and timeout
   - `rpc.log` routed as text when detached and as frames when attached
   - `on_text`
4. Merge `serialport.hpp` into a header-only version and build it on macOS.
5. `serial_rpc.hpp` + a test that runs the host `RPC` against the device `Server` over an in-memory pipe, templating `RPC` on the port type.
6. `Blink.ino` + `rpc_cli`, then compile with arduino-cli for AVR.

## Verification
- **Unit tests:** `cmake -Bbuild -G Ninja && cmake --build build && ctest --test-dir build`. These cover the codec, the framing, the server dispatch, and an end-to-end in-memory host⇄device loop.
- **Unit tests for both backends:** the `serial_rpc_tests_fixed` and `serial_rpc_tests_stl` targets both run under `ctest`.
- **AVR build:** `arduino-cli compile -b arduino:avr:uno ...` must succeed. Check the flash and RAM report, aiming for under 1 KB of RAM on the Uno for the example.
- **STL build:** compile the same Blink example plus an `examples/StlFeatures` sketch for one 32-bit core that is installed, e.g. `esp32:esp32:esp32` or `arduino:samd:mkrzero`. Confirm that `StlFeatures` fails on AVR with the intended `static_assert` message.
- **Hardware smoke test** (if a board is attached): upload Blink, then run:
  - `rpc_cli -p <port> rpc.list`
  - `rpc_cli -p <port> set_led 13 true`, and confirm the LED toggles
  - `rpc_cli -p <port> get_temp`
  - `rpc_cli -p <port> --monitor`, which attaches and prints framed logs, events and raw text
  - With the host detached, the Arduino IDE Serial Monitor shows clean text logs.
- **Portability:** the host headers must compile warning-free with Clang at `-Wall -Wextra`. The Windows path is compile-checked only by review unless an MSVC environment is available.
