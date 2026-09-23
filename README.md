<div align="center">

<img src="images/serial-rpc-banner.png" alt="SerialRPC connects an Arduino and a host computer over one shared serial link" width="100%">

# SerialRPC

### Your serial port, upgraded.

Typed calls, notifications, and logs between Arduino and a host computer — while ordinary `Serial.print()` output keeps working on the very same wire.

[![CI](https://github.com/pbosetti/arduino_rpc/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/pbosetti/arduino_rpc/actions/workflows/ci.yml)
[![Arduino](https://img.shields.io/badge/Arduino-C%2B%2B11-00878F?logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![Host](https://img.shields.io/badge/Host-C%2B%2B20-00599C?logo=cplusplus&logoColor=white)](#host-library)
[![License](https://img.shields.io/badge/license-Apache--2.0-D22128)](LICENSE)

**No heap on AVR · No runtime dependencies · MessagePack-RPC · COBS + CRC16 · Header-only host client**

[Quick start](#quick-start) · [`rpc_repl`](#meet-rpc_repl) · [Host API](#host-library) · [Protocol](#one-wire-two-worlds) · [Design notes](extras/docs/PLAN.md)

</div>

---

SerialRPC makes an Arduino feel like a tiny typed service:

```cpp
rpc.bind("set_led", set_led);
rpc.bind("set_period", [](uint32_t ms) { period_ms = ms; });
rpc.bind("get_count", counter, &Counter::value);
```

Call those methods from C++ or a terminal:

```console
$ rpc_repl -p /dev/cu.usbmodem1101 set_led 13 true
null
$ rpc_repl -p /dev/cu.usbmodem1101 get_count
42
```

No hand-written parser. No stringly typed protocol. No giving up the Serial Monitor.

## Why SerialRPC?

| | What you get |
|---|---|
| **Typed by default** | Bind free functions, lambdas, and member functions. Arguments are decoded and checked for you. |
| **Text and RPC together** | Logs, human-entered commands, and binary RPC frames safely share one serial connection. |
| **Small-board friendly** | The Arduino path is C++11, fixed-capacity, dependency-free, and heap-free on AVR. |
| **Reliable on a noisy stream** | COBS framing, CRC16 validation, and automatic resynchronization keep packets intact. |
| **Useful from day one** | A header-only C++20 client plus `rpc_repl` for interactive sessions, scripts, and monitoring. |
| **Works across boards** | Tested in CI on the classic Uno and UNO R4 Minima; portable across Arduino architectures. |

## Quick start

### 1. Expose a method on Arduino

The repository root is the Arduino library, so the bundled example compiles directly with `--library .`:

```sh
arduino-cli compile -b arduino:avr:uno --library . examples/Blink
arduino-cli upload -b arduino:avr:uno -p <port> --library . examples/Blink
```

The essential sketch is deliberately small:

```cpp
#include <SerialRPC.h>

void set_led(uint8_t pin, bool on) {
  digitalWrite(pin, on ? HIGH : LOW);
}

serial_rpc::SerialRPC<5, 96> rpc(Serial);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  rpc.begin();
  rpc.bind("set_led", set_led);
}

void loop() {
  rpc.poll(); // non-blocking
}
```

See [`examples/Blink/Blink.ino`](examples/Blink/Blink.ino) for free functions, capture-less and capturing lambdas, member functions, notifications, framed logging, and plain-text input in one sketch.

### 2. Build the host tools

```sh
cmake -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```

The build produces `rpc_repl`, the header-only `serial_rpc` interface target, and the test executables. CMake fetches dependencies used by the tests and REPL; the Arduino and host libraries themselves remain dependency-free.

### 3. Talk to the board

```console
$ ./build/rpc_repl -p /dev/cu.usbmodem1101
rpc> .list
rpc> set_led 13 true
rpc> get_status
"led=on"
```

If exactly one likely board is connected, omit `-p` and SerialRPC will find it.

## One wire, two worlds

The device starts **detached**, so it behaves like a normal sketch: `rpc.log` is plain text and the Arduino Serial Monitor remains useful. When a host attaches, notifications and structured logs become framed messages. Regular `Serial.print()` output is still accepted in either mode.

```text
plain text ──────────────┐
                         ├── one serial stream ──► Demux ──► text lines
RPC request / response ──┤                              └──► verified frames
events + framed logs ────┘
```

On the wire, RPC uses standard MessagePack-RPC message shapes inside zero-delimited COBS frames protected by CRC16-CCITT:

```text
0x00 · COBS(MessagePack payload + CRC16) · 0x00
```

Because COBS output contains no zero bytes, frames and printable text are easy to distinguish byte by byte. The shared demultiplexer also recovers automatically after truncated or corrupt input. The only rule: do not send arbitrary zero-containing binary with raw `Serial.write()` on the same port.

## Meet `rpc_repl`

Use the REPL interactively, as a one-shot command in a script, or as a live monitor:

```sh
rpc_repl [-p PORT] [-b BAUD=115200] [-t TIMEOUT_MS=1000] \
         [--no-color] [--no-attach] [--monitor] [method args...]
```

Arguments are JSON when possible and plain strings otherwise. Arrays and objects may contain spaces:

```console
rpc> set_period 250
rpc> cfg {"kp": 1.5, "ki": [0, 1]}
rpc> echo "13"
```

The interactive shell provides persistent history, method completion, signature hints, timestamps, color-coded traffic, round-trip timing, and automatic reconnection when a board is unplugged or reflashed.

| Command | Effect |
|---|---|
| `.list` | Show method names and signatures |
| `.attach` / `.detach` | Enable or disable unsolicited framed traffic |
| `.notify <method> [args...]` | Send a fire-and-forget call |
| `.text <line>` | Send a raw text line |
| `.filter [+\|-]log\|evt\|txt` | Show or hide traffic classes |
| `.reset [hard]` | Pulse DTR, or perform a 1200-baud touch reset |
| `.reconnect` | Close and reopen the port |
| `.timeout <ms>` | Change the call timeout |
| `.help` / `.quit` | Show help or exit |

One-shot mode prints JSON to stdout and uses meaningful exit codes: `0` success, `1` remote error, `2` port selection, `3` timeout, and `4` I/O error.

## Host library

The host client is a dependency-free, header-only C++20 API built on the included portable serial port:

```cpp
#include "serial_rpc.hpp"

SerialPort port("/dev/cu.usbmodem1101", {.baud_rate = 115200});
serial_rpc::RPC<SerialPort> rpc(port);

rpc.connect();

rpc.call("set_led", 13, true);
int count = rpc.call<int>("get_count");
rpc.notify("set_period", 250);

rpc.on("tick", [](const serial_rpc::Value &args) {
  // Handle a device notification.
});

rpc.disconnect();
```

`Value` represents decoded nil, boolean, integer, floating-point, string, binary, array, and map values. Use `call<T>()` for conversion to a known type or `call()` to keep the dynamic value. The API reports remote failures and timeouts with dedicated exceptions and exposes callbacks for notifications, framed logs, and plain text.

Full API and threading details live in [`extras/host/include/serial_rpc.hpp`](extras/host/include/serial_rpc.hpp).

## Arduino API at a glance

```cpp
rpc.begin();                              // announce ready; start detached
rpc.poll();                               // drain and dispatch without blocking
rpc.bind("name", callable);               // typed free function or lambda
rpc.bind("name", object, &Type::method);  // typed member function
rpc.bind_raw("name", handler);            // complex/variable argument escape hatch
rpc.notify("event", value);               // device → host notification
rpc.log.println("temperature stable");    // text or framed log, by mode
rpc.on_text(handler);                     // receive human-readable lines
```

On AVR, handlers use fixed-capacity storage with no heap. Supported 32-bit boards can automatically use an STL-backed implementation for richer argument types and owning captures; the common API stays the same. Buffer and handler limits remain explicit template parameters when tight RAM budgets matter.

## Repository map

```text
src/                         Arduino library + shared codec and framing
examples/Blink/              Complete device example
extras/host/include/         Header-only host RPC and serial-port clients
extras/host/tools/rpc_repl/  Interactive and one-shot terminal client
extras/tests/                Codec, framing, RPC, server, and REPL tests
extras/docs/PLAN.md          Protocol and implementation design
```

Everything outside the Arduino library lives under `extras/`, which Arduino tooling ignores. There is one source of truth for MessagePack and framing: the host consumes the same headers as the device.

## Install into an Arduino sketchbook

To copy the library and example into the sketchbook reported by `arduino-cli`:

```sh
cmake -B build
cmake --build build --target arduino_install
```

Override the destination with `-DARDUINO_USER_DIR=/path/to/sketchbook`. The target replaces an older `SerialRPC` library installation but preserves an existing `SerialRPC_Blink` sketch so local edits are not lost.

## License

SerialRPC is released under the [Apache License 2.0](LICENSE). Third-party build and REPL dependencies retain their own licenses.

---

<div align="center">

**Make the wire disappear. Call the function.**

</div>
