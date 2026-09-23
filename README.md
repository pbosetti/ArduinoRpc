# arduino_rpc

Typed request/response and notification RPC between a host PC and an
Arduino, over a single serial line shared with plain text -- using a small,
dependency-free MsgPack codec of its own. Full design in
[`extras/docs/PLAN.md`](extras/docs/PLAN.md).

The repository root **is** the `SerialRPC` Arduino library (1.5 format,
installable via the Arduino Library Manager or `arduino-cli --library`).
Everything that isn't part of the Arduino library -- the host build, tools,
and tests -- lives under [`extras/`](extras/), which Arduino's tooling
ignores.

- **Arduino side** (`src/`): a C++11, no-heap library. Bind a
  free function, a lambda, or a member function to a name; `rpc.poll()` in
  `loop()` dispatches incoming calls. Plain `Serial.print()` and RPC frames
  coexist on the same line -- a `Demux` tells them apart byte by byte, so
  the Arduino Serial Monitor looks like an ordinary sketch until a host
  attaches.
- **Host side** (`extras/host/include/`): header-only C++20 --
  `serialport.hpp` (a portable serial port) and `serial_rpc.hpp` (the
  `RPC<Port>` client).
- **`rpc_repl`** (`extras/host/tools/rpc_repl/`): an interactive REPL and
  one-shot CLI for talking to a device from a terminal or a script.

## Build

```sh
cmake -Bbuild -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```

This fetches `doctest`, `fmt`, `cxxopts`, `nlohmann_json` and `replxx` via
CMake `FetchContent` (the last four only for `rpc_repl`; the Arduino and
host libraries stay dependency-free). Targets: `serial_rpc` (the header-only
libraries, as an INTERFACE target), `rpc_repl`, and the `*_tests`
executables `ctest` runs.

## Arduino quick start: Blink

The repo root is the library itself, so `--library .` (or `--library
/path/to/arduino_rpc`) is all `arduino-cli` needs -- no separate install
step. Run from the repo root:

```sh
arduino-cli compile -b arduino:avr:uno --library . examples/Blink
arduino-cli upload -b arduino:avr:uno -p <port> --library . examples/Blink
```

Alternatively, install it into your sketchbook the way the Arduino IDE /
Library Manager does, after which `--library` is no longer needed:

```sh
cmake -Bbuild && cmake --build build --target arduino_install
```

This copies the library to `<sketchbook>/libraries/SerialRPC` (replacing a
previous SerialRPC install; its examples show up under File > Examples >
SerialRPC) and the Blink example to `<sketchbook>/SerialRPC_Blink` (only if
that sketch doesn't exist yet, so your edits are kept). The sketchbook is
the one `arduino-cli config get directories.user` reports; override it with
`cmake -Bbuild -DARDUINO_USER_DIR=/path/to/sketchbook`.

The sketch ([`examples/Blink/Blink.ino`](examples/Blink/Blink.ino)),
in outline:

```cpp
#include <SerialRPC.h>

void set_led(uint8_t pin, bool on) { digitalWrite(pin, on ? HIGH : LOW); }

serial_rpc::SerialRPC<5, 96> rpc(Serial); // 5 handlers, 96-byte frame budget

void setup() {
  Serial.begin(115200);
  rpc.begin();                                                  // prints "ready"; starts detached
  rpc.bind("set_led", set_led);                                 // free function
  rpc.bind("set_period", [](uint32_t ms) { g_period_ms = ms; }); // capture-less lambda
  rpc.bind("get_count", counter, &Counter::value);              // member function
}

void loop() {
  rpc.poll();               // dispatch incoming calls, non-blocking
  // ... blink the LED, then:
  rpc.notify("tick", count); // a no-op while detached, an event once attached
  rpc.log.println("status: ...");  // plain text detached, a framed log once attached
}
```

Open the Arduino Serial Monitor at 115200 baud and it looks like a normal
sketch -- or point `rpc_repl` at the port (below) to call `set_led`,
`get_status`, `get_count`, `reset_counter`, watch `tick` events, and see
`rpc.log` lines framed instead of plain.

## Host library

```cpp
#include "serial_rpc.hpp"

SerialPort port("/dev/cu.usbmodem1101", {.baud_rate = 115200});
serial_rpc::RPC<SerialPort> rpc(port);

rpc.connect();                        // wait for "ready"/a ping reply, then rpc.attach
int status = rpc.call<int>("get_count");
rpc.call("set_led", 13, true);
rpc.notify("set_period", 250);        // fire-and-forget
rpc.on("tick", [](const serial_rpc::Value &args) { /* ... */ });
rpc.disconnect();                     // sends rpc.detach; also runs in the destructor
```

`Value` is the dynamically-typed decoded form (nil/bool/int/uint/double/
string/bin/array/map); `call<int>(...)` converts it, `call(...)` returns it
as-is. See the doc comments in
[`extras/host/include/serial_rpc.hpp`](extras/host/include/serial_rpc.hpp) for the full
API, error types, and threading notes.

## `rpc_repl`

```sh
rpc_repl [-p PORT] [-b BAUD=115200] [-t TIMEOUT_MS=1000] [--no-color] [--no-attach] [--monitor] [method args...]
```

- No `method`: an interactive REPL (history in `~/.rpc_repl_history`, Tab
  completion over method names and meta-commands, hints showing a method's
  signature as you type it).
- A `method [args...]`: makes one call, prints the result as JSON on
  stdout, and exits 0 (ok) / 1 (remote error) / 3 (timeout) / 4 (I/O error).
- `--monitor`: attaches and prints traffic until Ctrl-C.
- `-p` can be omitted if exactly one port looks like a board
  (`usbmodem`/`usbserial`/`ttyACM`/`ttyUSB`/`COM`); otherwise `rpc_repl`
  lists the candidates and exits 2.

```sh
rpc_repl -p /dev/cu.usbmodem1101 get_status
rpc_repl -p /dev/cu.usbmodem1101 set_led 13 true
rpc_repl -p /dev/cu.usbmodem1101          # interactive
rpc> set_led 13 true
rpc> cfg {"kp": 1.5, "ki": [0, 1]}
```

**Call syntax:** `<method> [args...]`. Each argument is parsed as JSON
(`nlohmann/json`) and falls back to a plain string if it doesn't parse;
`[...]`/`{...}` brackets span spaces as one argument, so
`cfg {"kp": 1.5, "ki": [0, 1]}` sends one map argument. Quoted arguments
(`"..."` or `'...'`) are always strings, even if JSON-shaped (`"13"` stays
the string `"13"`).

**Meta-commands:**

| Command | Effect |
|---|---|
| `.help` | show help |
| `.list` | method names and signatures |
| `.attach` / `.detach` | turn the framed protocol on/off |
| `.notify <method> [args...]` | fire-and-forget call |
| `.text <line>` | send a raw text line, verbatim |
| `.reset [hard]` | pulse DTR, or (`hard`) a 1200-baud touch reset |
| `.reconnect` | close and reopen the port |
| `.filter [+\|-]log\|evt\|txt` | show/toggle a message filter |
| `.timeout <ms>` | set the call timeout |
| `.quit` | exit (Ctrl-D also works) |

Output is timestamped (`HH:MM:SS.mmm`) and colour-coded (`--no-color` or a
non-TTY stdout disables colour): `[log L]` framed logs by level, `[evt]`
notifications, `[txt]` raw text (dimmed), `→` outgoing calls, `←` results
(with round-trip time) or errors, `[sys]` connection/system messages. A
background thread owns the port and keeps polling and reconnecting (every
500 ms while the port is unplugged or unresponsive) while the REPL stays
responsive.

## License

Apache License 2.0 -- see [`LICENSE`](LICENSE). Each of this project's own
source files carries an `SPDX-License-Identifier: Apache-2.0` line;
third-party dependencies pulled in by the build (doctest, fmt, cxxopts,
nlohmann/json, replxx) keep their own licenses.
