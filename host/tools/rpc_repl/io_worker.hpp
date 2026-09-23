/// \file io_worker.hpp
/// \brief `IoWorker`: owns the `SerialPort` + `RPC<SerialPort>` pair on its
///   own thread (docs/PLAN.md component 7). The REPL thread never touches
///   the port -- it only pushes `Command`s through `IoWorker::submit()` and
///   receives formatted output through the `OutputSink` callback given to
///   the constructor.
#pragma once

#include "serial_rpc.hpp"

#include <fmt/format.h>

#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <variant>
#include <vector>

namespace rpc_repl {

/// Kind of one line of output, for the REPL's colouring/prefixing
/// (docs/PLAN.md's "Output" bullets: `[log L]`, `[evt]`, `[txt]`, `->`,
/// `<-`, `[sys]`). `IoWorker` decides the Kind; the console layer (repl.hpp)
/// decides how to render it.
///
/// `OutputSink` is deliberately just `(Kind, string)`, per docs/PLAN.md --
/// but `Kind::Log` needs a severity level to colour by, so its message is
/// formatted as `"<level>|<line>"` (see `emit_log_()`); the console layer
/// splits on the first `|` to recover the level and the line text.
enum class Kind { Log, Event, Text, Outgoing, Result, Error, Sys };

using OutputSink = std::function<void(Kind, std::string)>;
using MethodsSink = std::function<void(std::vector<serial_rpc::MethodInfo>)>;

/// Formats a microsecond duration (`RPC::last_rtt()`'s unit -- USB round
/// trips are commonly sub-millisecond) as e.g. "0.6 ms" or "12.3 ms".
inline std::string format_ms(std::chrono::microseconds us) {
  return fmt::format("{:.1f} ms", static_cast<double>(us.count()) / 1000.0);
}

// ===========================================================================
// Commands queued from the REPL thread, executed on the I/O thread.
// ===========================================================================

struct CallCmd {
  std::string method;
  std::vector<serial_rpc::Value> args;
};
struct NotifyCmd {
  std::string method;
  std::vector<serial_rpc::Value> args;
};
struct SendTextCmd {
  std::string line;
};
struct AttachCmd {};
struct DetachCmd {};
struct ListCmd {};
struct ResetCmd {
  bool hard = false;
};
struct ReconnectCmd {};
struct SetTimeoutCmd {
  std::chrono::milliseconds value;
};

using Command = std::variant<CallCmd, NotifyCmd, SendTextCmd, AttachCmd, DetachCmd, ListCmd, ResetCmd, ReconnectCmd,
                              SetTimeoutCmd>;

struct IoWorkerOptions {
  std::string port_path;
  unsigned baud = 115200;
  std::chrono::milliseconds call_timeout{1000};
  /// If true, the worker attaches automatically after every successful
  /// open (initial and after a reconnect): waits for "ready"/a ping reply,
  /// then `rpc.attach`, then refreshes the method list. `--no-attach`
  /// clears this.
  bool auto_attach = true;
};

/// See docs/PLAN.md component 7 ("Threads", "Resilience"). Not copyable or
/// movable -- own it by value or behind a pointer, not both.
class IoWorker {
public:
  using Clock = std::chrono::steady_clock;

  IoWorker(IoWorkerOptions opts, OutputSink sink, MethodsSink methods_sink)
      : _opts(std::move(opts)), _sink(std::move(sink)), _methods_sink(std::move(methods_sink)) {}

  ~IoWorker() { stop(); }

  IoWorker(const IoWorker &) = delete;
  IoWorker &operator=(const IoWorker &) = delete;

  /// Spawns the I/O thread. Call once.
  void start() {
    _stop.store(false);
    _thread = std::thread([this] { thread_main_(); });
  }

  /// Signals the thread to stop and joins it (idempotent). Disconnects
  /// cleanly (an `rpc.detach` is sent if attached) before the port closes.
  void stop() {
    _stop.store(true);
    if (_thread.joinable()) _thread.join();
  }

  /// Thread-safe: queues `cmd` for the I/O thread to execute. Never blocks
  /// on I/O itself -- the REPL thread returns immediately.
  void submit(Command cmd) {
    std::lock_guard<std::mutex> lock(_queue_mu);
    _queue.push_back(std::move(cmd));
  }

  bool attached() const noexcept { return _attached.load(); }
  bool connected() const noexcept { return _connected.load(); }

private:
  static constexpr auto kPollInterval = std::chrono::milliseconds(20);
  static constexpr auto kReopenInterval = std::chrono::milliseconds(500);
  static constexpr auto kConnectWait = std::chrono::milliseconds(3000);
  static constexpr auto kResetPulseGap = std::chrono::milliseconds(100);
  static constexpr auto kTouchSettle = std::chrono::milliseconds(400);
  static constexpr unsigned kTouchBaud = 1200;

  void thread_main_() {
    while (!_stop.load()) {
      drain_queue_();
      if (_stop.load()) break;

      if (!_port) {
        const auto now = Clock::now();
        if (now >= _next_open_attempt) {
          try_open_();
          _next_open_attempt = Clock::now() + kReopenInterval;
        }
        std::this_thread::sleep_for(kPollInterval);
        continue;
      }

      try {
        _rpc->poll(kPollInterval);
      } catch (const serial_rpc::IoError &e) {
        handle_io_error_(e.what());
      } catch (const serial_rpc::ProtocolError &e) {
        emit_(Kind::Sys, fmt::format("protocol error: {}", e.what()));
      }
    }
    close_(/*send_detach=*/true);
  }

  void drain_queue_() {
    for (;;) {
      Command cmd;
      {
        std::lock_guard<std::mutex> lock(_queue_mu);
        if (_queue.empty()) return;
        cmd = std::move(_queue.front());
        _queue.pop_front();
      }
      execute_(cmd);
      if (_stop.load()) return;
    }
  }

  // --- connection lifecycle -------------------------------------------

  void setup_rpc_callbacks_() {
    _rpc->on_log([this](int level, std::string_view line) { emit_(Kind::Log, fmt::format("{}|{}", level, line)); });
    _rpc->on_text([this](std::string_view line) { emit_(Kind::Text, std::string(line)); });
    _rpc->on_any_notification([this](std::string_view method, const serial_rpc::Value &args) {
      emit_(Kind::Event, fmt::format("{} {}", method, args.to_string()));
    });
  }

  void try_open_() {
    try {
      auto port = std::make_unique<SerialPort>(
          _opts.port_path, SerialPort::Config{.baud_rate = _opts.baud, .timeout = SerialPort::Milliseconds{50}});
      _port = std::move(port);
      _rpc = std::make_unique<serial_rpc::RPC<SerialPort>>(*_port,
                                                             serial_rpc::RPC<SerialPort>::Options{.timeout = _opts.call_timeout});
      setup_rpc_callbacks_();
      _connected.store(true);
      _last_open_error.clear();
      emit_(Kind::Sys, fmt::format("connected to {} @ {} baud", _opts.port_path, _opts.baud));
      if (_opts.auto_attach) attach_(/*from_reset=*/false);
    } catch (const std::exception &e) {
      _rpc.reset();
      _port.reset();
      // Retried every 500ms while unplugged: only log when the failure
      // reason changes, so a long-unplugged board doesn't spam the console.
      if (_last_open_error != e.what()) {
        _last_open_error = e.what();
        emit_(Kind::Sys, fmt::format("open {} failed: {}", _opts.port_path, e.what()));
      }
    }
  }

  /// Waits for the device (optionally already just DTR/1200-baud reset) and
  /// sends `rpc.attach`, then refreshes the method list. Used by the
  /// initial auto-attach, `.attach`, and after a `.reset`.
  void attach_(bool from_reset) {
    if (!_rpc) {
      emit_(Kind::Error, "not connected: no open port");
      return;
    }
    try {
      _rpc->connect(kConnectWait, /*reset=*/false);
      _attached.store(true);
      emit_(Kind::Sys, "attached");
      refresh_methods_();
    } catch (const serial_rpc::TimeoutError &e) {
      const char *hint = from_reset ? " (device may not print \"ready\" or answer rpc.ping after this reset)" : "";
      emit_(Kind::Sys, fmt::format("attach failed: {}{}", e.what(), hint));
    } catch (const serial_rpc::IoError &e) {
      handle_io_error_(e.what());
    }
  }

  void refresh_methods_() {
    if (!_rpc) return;
    try {
      std::vector<serial_rpc::MethodInfo> methods = _rpc->list();
      if (_methods_sink) _methods_sink(methods);
    } catch (const std::exception &e) {
      emit_(Kind::Sys, fmt::format("rpc.list failed: {}", e.what()));
    }
  }

  /// Drops the port/RPC object. `send_detach` also sends `rpc.detach` first
  /// if attached (best-effort: swallows I/O errors, since a broken port is
  /// exactly why this is often called). Used by shutdown, `.reconnect`,
  /// and the 1200-baud touch reset -- but *not* by `handle_io_error_()`,
  /// which already knows the port is broken and would just get another
  /// IoError trying to write to it.
  void close_(bool send_detach) {
    if (_rpc && send_detach) {
      try {
        _rpc->disconnect();
      } catch (...) {
        // Best-effort only; the port may already be gone.
      }
    }
    _rpc.reset();
    _port.reset();
    _attached.store(false);
    _connected.store(false);
  }

  void handle_io_error_(std::string_view what) {
    emit_(Kind::Sys, fmt::format("I/O error: {}; closing port and retrying every {} ms", what, kReopenInterval.count()));
    _rpc.reset();
    _port.reset();
    _attached.store(false);
    _connected.store(false);
    _last_open_error.clear();
    // Deliberately *not* an instant retry: a port that opens fine at the OS
    // level but never behaves (wrong baud, a wedged device, ...) would
    // otherwise open/fail/close in a tight, unthrottled loop -- every
    // failure here follows the same kReopenInterval cadence as an outright
    // closed port.
    _next_open_attempt = Clock::now() + kReopenInterval;
  }

  // --- command execution -------------------------------------------------

  void execute_(Command &cmd) {
    std::visit(
        [this](auto &c) {
          using T = std::decay_t<decltype(c)>;
          if constexpr (std::is_same_v<T, CallCmd>) execute_call_(c);
          else if constexpr (std::is_same_v<T, NotifyCmd>) execute_notify_(c);
          else if constexpr (std::is_same_v<T, SendTextCmd>) execute_send_text_(c);
          else if constexpr (std::is_same_v<T, AttachCmd>) attach_(/*from_reset=*/false);
          else if constexpr (std::is_same_v<T, DetachCmd>) execute_detach_();
          else if constexpr (std::is_same_v<T, ListCmd>) execute_list_();
          else if constexpr (std::is_same_v<T, ResetCmd>) execute_reset_(c);
          else if constexpr (std::is_same_v<T, ReconnectCmd>) execute_reconnect_();
          else if constexpr (std::is_same_v<T, SetTimeoutCmd>) execute_set_timeout_(c);
        },
        cmd);
  }

  void execute_call_(const CallCmd &c) {
    if (!_rpc) {
      emit_(Kind::Error, "not connected: no open port");
      return;
    }
    emit_(Kind::Outgoing, fmt::format("call {}({})", c.method, join_args_(c.args)));
    try {
      serial_rpc::Value result = _rpc->call_values(c.method, c.args);
      emit_(Kind::Result, fmt::format("{} ({})", result.to_string(), format_ms(_rpc->last_rtt())));
    } catch (const serial_rpc::RemoteError &e) {
      emit_(Kind::Error, fmt::format("{}: {}", c.method, e.error_value().to_string()));
    } catch (const serial_rpc::TimeoutError &e) {
      emit_(Kind::Error, e.what());
    } catch (const serial_rpc::IoError &e) {
      emit_(Kind::Error, e.what());
      handle_io_error_(e.what());
    } catch (const serial_rpc::ProtocolError &e) {
      emit_(Kind::Error, e.what());
    }
  }

  void execute_notify_(const NotifyCmd &c) {
    if (!_rpc) {
      emit_(Kind::Error, "not connected: no open port");
      return;
    }
    emit_(Kind::Outgoing, fmt::format("notify {}({})", c.method, join_args_(c.args)));
    try {
      _rpc->notify_values(c.method, c.args);
    } catch (const serial_rpc::IoError &e) {
      emit_(Kind::Error, e.what());
      handle_io_error_(e.what());
    } catch (const serial_rpc::ProtocolError &e) {
      emit_(Kind::Error, e.what());
    }
  }

  void execute_send_text_(const SendTextCmd &c) {
    if (!_rpc) {
      emit_(Kind::Error, "not connected: no open port");
      return;
    }
    emit_(Kind::Outgoing, fmt::format("text {}", c.line));
    try {
      _rpc->send_text(c.line);
    } catch (const serial_rpc::IoError &e) {
      emit_(Kind::Error, e.what());
      handle_io_error_(e.what());
    }
  }

  void execute_detach_() {
    if (!_rpc) {
      emit_(Kind::Error, "not connected: no open port");
      return;
    }
    try {
      _rpc->disconnect();
      _attached.store(false);
      emit_(Kind::Sys, "detached");
    } catch (const serial_rpc::IoError &e) {
      emit_(Kind::Error, e.what());
      handle_io_error_(e.what());
    }
  }

  void execute_list_() {
    if (!_rpc) {
      emit_(Kind::Error, "not connected: no open port");
      return;
    }
    refresh_methods_();
  }

  /// Plain `.reset` pulses DTR (works on boards with a DTR-wired auto-reset
  /// circuit, e.g. an Uno). `.reset hard` does the 1200-baud touch instead,
  /// for native-USB boards (e.g. UNO R4) that don't reset on DTR: open the
  /// port at 1200 baud, close it, wait, then let the normal reopen/attach
  /// path pick the board back up once it re-enumerates.
  void execute_reset_(const ResetCmd &c) {
    if (!c.hard) {
      if (!_port) {
        emit_(Kind::Error, "not connected: no open port");
        return;
      }
      _port->set_dtr(false);
      std::this_thread::sleep_for(kResetPulseGap);
      _port->set_dtr(true);
      emit_(Kind::Sys, "DTR pulsed");
      if (_opts.auto_attach) attach_(/*from_reset=*/true);
      return;
    }

    emit_(Kind::Sys, "performing a 1200-baud touch reset...");
    const std::string path = _opts.port_path;
    close_(/*send_detach=*/true);
    try {
      SerialPort touch(path, SerialPort::Config{.baud_rate = kTouchBaud, .timeout = SerialPort::Milliseconds{200}});
      touch.close();
    } catch (const std::exception &e) {
      emit_(Kind::Sys, fmt::format("1200-baud touch open failed: {} (the board may still reset)", e.what()));
    }
    std::this_thread::sleep_for(kTouchSettle);
    _next_open_attempt = Clock::now(); // let the main loop reopen it right away
  }

  void execute_reconnect_() {
    close_(/*send_detach=*/true);
    _next_open_attempt = Clock::now();
    emit_(Kind::Sys, "reconnecting...");
  }

  /// `RPC<Port>` has no public setter for its call timeout, so changing it
  /// means rebuilding the RPC object on the same still-open port. If it was
  /// attached, the old object's destructor sends a real `rpc.detach` and
  /// this then sends a fresh `rpc.attach` -- a genuine, harmless
  /// detach/reattach cycle, not a workaround.
  void execute_set_timeout_(const SetTimeoutCmd &c) {
    _opts.call_timeout = c.value;
    if (!_port) {
      emit_(Kind::Sys, fmt::format("call timeout set to {} ms (applies once connected)", c.value.count()));
      return;
    }
    const bool was_attached = _attached.load();
    _rpc.reset(); // sends rpc.detach if it was attached (best-effort, swallowed by ~RPC)
    _attached.store(false);
    _rpc = std::make_unique<serial_rpc::RPC<SerialPort>>(*_port, serial_rpc::RPC<SerialPort>::Options{.timeout = c.value});
    setup_rpc_callbacks_();
    emit_(Kind::Sys, fmt::format("call timeout set to {} ms", c.value.count()));
    if (was_attached) attach_(/*from_reset=*/false);
  }

  // --- output --------------------------------------------------------

  void emit_(Kind k, std::string msg) {
    if (_sink) _sink(k, std::move(msg));
  }

  static std::string join_args_(const std::vector<serial_rpc::Value> &args) {
    std::string out;
    for (size_t i = 0; i < args.size(); ++i) {
      if (i) out += ", ";
      out += args[i].to_string();
    }
    return out;
  }

  IoWorkerOptions _opts;
  OutputSink _sink;
  MethodsSink _methods_sink;

  std::thread _thread;
  std::atomic<bool> _stop{true};
  std::atomic<bool> _attached{false};
  std::atomic<bool> _connected{false};

  std::mutex _queue_mu;
  std::deque<Command> _queue;

  std::unique_ptr<SerialPort> _port;
  std::unique_ptr<serial_rpc::RPC<SerialPort>> _rpc;

  Clock::time_point _next_open_attempt{};
  std::string _last_open_error;
};

} // namespace rpc_repl
