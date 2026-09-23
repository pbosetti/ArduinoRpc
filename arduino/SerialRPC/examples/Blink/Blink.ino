// Blink.ino - SerialRPC example (docs/PLAN.md, "Blink.ino + rpc_cli").
//
// A blinking LED whose period, on/off state and tick count are reachable
// over MsgPack-RPC, alongside plain-text logging and a human-typed command
// line -- all sharing the same serial port at once (see docs/PLAN.md's
// "Wire protocol" section for how text and RPC frames coexist).
//
// Binds one of each kind of callable `bind()` supports:
//  - a free function                     (set_led)
//  - a capture-less lambda               (set_period)
//  - a lambda capturing state            (reset_counter, over `counter`)
//  - a member function                   (Counter::value, const)
//  - a free function with a return value (get_status)
//
// Try it with the Arduino Serial Monitor (line ending "Newline", 115200
// baud): typed lines are echoed back by `on_text`, and `rpc.log` lines show
// up as plain text -- because the device starts detached, nothing here
// looks any different from an ordinary sketch until a host RPC client
// attaches.
#include <SerialRPC.h>

namespace {

const uint8_t kLedPin = LED_BUILTIN;

void set_led(uint8_t pin, bool on) { digitalWrite(pin, on ? HIGH : LOW); }

// Plain globals read/written by a capture-less lambda (docs/PLAN.md's own
// bind() example calls this shape "capture-less (globals)").
uint32_t g_period_ms = 500;
bool g_led_on = false;
uint32_t g_last_toggle_ms = 0;

/// A small class bound both via a capturing lambda and via a
/// member-function pointer (see setup(), below).
class Counter {
public:
  void reset() { _count = 0; }
  int32_t increment() { return ++_count; }
  int32_t value() const { return _count; }

private:
  int32_t _count = 0;
};
Counter counter;

const char *get_status() { return g_led_on ? "led=on" : "led=off"; }

} // namespace

// 6 handlers (this sketch binds 5) and a 96-byte frame/payload budget (this
// sketch's requests and replies are all a handful of bytes) comfortably fit
// every method below while keeping the Uno's RAM budget under 1 KB; see the
// budget breakdown on serial_rpc::Server's class comment. Bigger sketches
// with more methods or larger payloads can just use the SerialRPC<> default.
serial_rpc::SerialRPC<6, 96> rpc(Serial);

uint32_t last_log_ms = 0;

void setup() {
  pinMode(kLedPin, OUTPUT);
  Serial.begin(115200);
  rpc.begin(); // prints the "ready" text line; device starts detached

  rpc.bind("set_led", set_led);                                 // free function
  rpc.bind("set_period", [](uint32_t ms) { g_period_ms = ms; }); // capture-less lambda

  // A capturing lambda reaching `counter`, a small global class instance.
  // It captures a *local pointer to it*, by value -- legal for any
  // storage duration, exactly like `[this]` is (both just copy a pointer
  // value into the closure). Naming `counter` itself in the capture list
  // (`[&counter]`) would not compile: a lambda's capture list may only
  // name entities with automatic storage duration, or `this`; a global's
  // static storage duration doesn't qualify, which is exactly why
  // docs/PLAN.md's own note for this pattern is "inside a class:
  // rpc.bind(\"reset\", [this]() { reset(); });".
  Counter *const p_counter = &counter;
  rpc.bind("reset_counter", [p_counter]() { p_counter->reset(); });

  rpc.bind("get_count", counter, &Counter::value); // member function (const)
  rpc.bind("get_status", get_status);              // free function returning a value

  // Echo whatever a human types in the Serial Monitor back out, tagged --
  // demonstrates that plain text and RPC frames really do share the line.
  rpc.on_text([](const char *line) {
    rpc.log.print("echo: ");
    rpc.log.println(line);
  });
}

void loop() {
  rpc.poll();

  const uint32_t now = millis();
  if (now - g_last_toggle_ms >= g_period_ms) {
    g_last_toggle_ms = now;
    g_led_on = !g_led_on;
    set_led(kLedPin, g_led_on);
    const int32_t count = counter.increment();
    rpc.notify("tick", count); // a no-op (returns false) while detached
  }

  if (now - last_log_ms >= 5000) {
    last_log_ms = now;
    rpc.log.print("status: ");
    rpc.log.println(get_status());
  }
}
