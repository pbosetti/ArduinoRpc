// SPDX-License-Identifier: Apache-2.0
/// \file main.cpp
/// \brief `rpc_repl` entry point (docs/PLAN.md component 7): CLI parsing,
///   port auto-detection, and the three run modes -- one-shot (a method
///   given on the command line), `--monitor` (attach and print traffic
///   until Ctrl-C), and the interactive REPL.
#include "repl.hpp"

#include <cxxopts.hpp>
#include <fmt/color.h>
#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#if defined(_WIN32)
#include <io.h>
#else
#include <unistd.h>
#endif

namespace {

bool stdout_is_tty() {
#if defined(_WIN32)
  return _isatty(_fileno(stdout)) != 0;
#else
  return isatty(fileno(stdout)) != 0;
#endif
}

/// docs/PLAN.md: "-p is optional when SerialPort::available_ports() finds
/// exactly one likely board (usbmodem/usbserial/ttyACM/ttyUSB/COM)."
bool looks_like_board_port(const std::string &p) {
  return p.find("usbmodem") != std::string::npos || p.find("usbserial") != std::string::npos ||
         p.find("ttyACM") != std::string::npos || p.find("ttyUSB") != std::string::npos ||
         p.find("COM") != std::string::npos;
}

/// Returns the sole candidate port, or nullopt (having already printed a
/// diagnostic to stderr) if there isn't exactly one.
std::optional<std::string> auto_detect_port() {
  const std::vector<std::string> ports = SerialPort::available_ports();
  std::vector<std::string> candidates;
  for (const auto &p : ports) {
    if (looks_like_board_port(p)) candidates.push_back(p);
  }
  if (candidates.size() == 1) return candidates.front();

  if (candidates.empty()) {
    fmt::print(stderr, "rpc_repl: no port given (-p) and none could be auto-detected.\n");
    if (ports.empty()) {
      fmt::print(stderr, "No serial ports found at all.\n");
    } else {
      fmt::print(stderr, "Ports found, but none look like a board (usbmodem/usbserial/ttyACM/ttyUSB/COM):\n");
      for (const auto &p : ports) fmt::print(stderr, "  {}\n", p);
    }
  } else {
    fmt::print(stderr, "rpc_repl: -p not given and multiple candidate ports were found:\n");
    for (const auto &p : candidates) fmt::print(stderr, "  {}\n", p);
    fmt::print(stderr, "Pass -p <port> to pick one.\n");
  }
  return std::nullopt;
}

/// Converts one command-line argument to a `Value` the same way the REPL
/// treats a bare (unquoted) token (docs/PLAN.md): try it as JSON, and fall
/// back to a plain string if it doesn't parse. There's no shell-quoting or
/// bracket-spanning to do here -- the shell already split argv for us.
serial_rpc::Value arg_to_value(const std::string &s) {
  nlohmann::ordered_json parsed = nlohmann::ordered_json::parse(s, /*callback*/ nullptr, /*allow_exceptions*/ false);
  if (parsed.is_discarded()) return serial_rpc::Value(s);
  return rpc_repl::json_to_value(parsed);
}

/// One-shot mode: connect (no need to attach -- request/response calls
/// work whether the device is attached or not), make one call, print the
/// result as JSON on stdout. Exit codes per docs/PLAN.md: 0 ok, 1 remote
/// error, 3 timeout, 4 I/O (including a port that won't open) error.
int run_one_shot(const std::string &port_path, unsigned baud, int timeout_ms, const std::vector<std::string> &positional) {
  const std::string &method = positional.front();
  std::vector<serial_rpc::Value> args;
  args.reserve(positional.size() - 1);
  for (size_t i = 1; i < positional.size(); ++i) args.push_back(arg_to_value(positional[i]));

  try {
    SerialPort port(port_path, SerialPort::Config{.baud_rate = baud, .timeout = SerialPort::Milliseconds{50}});
    serial_rpc::RPC<SerialPort> rpc(port, serial_rpc::RPC<SerialPort>::Options{.timeout = std::chrono::milliseconds(timeout_ms)});
    const serial_rpc::Value result = rpc.call_values(method, args);
    std::cout << rpc_repl::value_to_json(result).dump() << "\n";
    return 0;
  } catch (const serial_rpc::RemoteError &e) {
    fmt::print(stderr, "rpc_repl: {}\n", e.error_value().to_string());
    return 1;
  } catch (const serial_rpc::TimeoutError &e) {
    fmt::print(stderr, "rpc_repl: {}\n", e.what());
    return 3;
  } catch (const serial_rpc::IoError &e) {
    fmt::print(stderr, "rpc_repl: {}\n", e.what());
    return 4;
  } catch (const std::exception &e) {
    // Port open failure (std::system_error / std::invalid_argument from the
    // SerialPort constructor) or a ProtocolError: I/O-ish, from a caller's
    // point of view, either way.
    fmt::print(stderr, "rpc_repl: {}\n", e.what());
    return 4;
  }
}

std::atomic<bool> g_stop_monitor{false};
extern "C" void handle_sigint(int) { g_stop_monitor.store(true); }

/// `--monitor`: attach and print traffic until Ctrl-C. No line editor is
/// involved, so (unlike the REPL, where replxx's raw mode intercepts
/// Ctrl-C itself) we install our own SIGINT handler to break the wait loop
/// and shut the I/O thread down cleanly instead of dying mid-write.
int run_monitor(const rpc_repl::IoWorkerOptions &opts, bool color) {
  std::signal(SIGINT, handle_sigint);

  std::mutex print_mu;
  rpc_repl::IoWorker worker(
      opts,
      [&](rpc_repl::Kind k, std::string msg) {
        std::lock_guard<std::mutex> lock(print_mu);
        std::cout << rpc_repl::format_output_line(k, msg, color) << "\n";
      },
      [](std::vector<serial_rpc::MethodInfo>) {});

  worker.start();
  while (!g_stop_monitor.load()) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  worker.stop();
  return 0;
}

int run_repl(const rpc_repl::IoWorkerOptions &opts, bool color) {
  // IoWorker's sink callbacks must exist at construction time, but they
  // need to call into the Repl, which itself needs a reference to a
  // (not-yet-started) IoWorker. Broken by forwarding through a pointer that
  // is filled in once the Repl exists, below -- safe because the sinks are
  // only ever invoked after worker.start(), which happens last.
  rpc_repl::Repl *repl_ptr = nullptr;
  rpc_repl::IoWorker worker(
      opts,
      [&repl_ptr](rpc_repl::Kind k, std::string msg) {
        if (repl_ptr) repl_ptr->print_output(k, std::move(msg));
      },
      [&repl_ptr](std::vector<serial_rpc::MethodInfo> methods) {
        if (repl_ptr) repl_ptr->set_methods(std::move(methods));
      });

  rpc_repl::Repl repl(worker, rpc_repl::ReplOptions{.color = color});
  repl_ptr = &repl;

  worker.start();
  const int rc = repl.run();
  worker.stop();
  return rc;
}

} // namespace

int main(int argc, char **argv) {
  cxxopts::Options options("rpc_repl",
                            "Interactive REPL / one-shot CLI for a serial_rpc device (docs/PLAN.md component 7)");
  // clang-format off
  options.add_options()
    ("p,port", "Serial port path (auto-detected if omitted and exactly one candidate is found)",
        cxxopts::value<std::string>())
    ("b,baud", "Baud rate", cxxopts::value<unsigned>()->default_value("115200"))
    ("t,timeout", "Call timeout, milliseconds", cxxopts::value<int>()->default_value("1000"))
    ("no-color", "Disable colored output")
    ("no-attach", "Don't attach automatically on connect")
    ("monitor", "Attach and print traffic until Ctrl-C")
    ("h,help", "Print usage")
    ("args", "method [args...]", cxxopts::value<std::vector<std::string>>())
  ;
  // clang-format on
  options.parse_positional({"args"});
  options.positional_help("[method args...]");

  cxxopts::ParseResult result;
  try {
    result = options.parse(argc, argv);
  } catch (const cxxopts::exceptions::exception &e) {
    fmt::print(stderr, "rpc_repl: {}\n", e.what());
    return 2;
  }

  if (result.count("help")) {
    std::cout << options.help() << "\n";
    return 0;
  }

  std::string port_path;
  if (result.count("port")) {
    port_path = result["port"].as<std::string>();
  } else {
    std::optional<std::string> detected = auto_detect_port();
    if (!detected) return 2;
    port_path = *detected;
  }

  const int timeout_ms = result["timeout"].as<int>();
  if (timeout_ms <= 0) {
    fmt::print(stderr, "rpc_repl: --timeout must be positive\n");
    return 2;
  }

  rpc_repl::IoWorkerOptions opts;
  opts.port_path = port_path;
  opts.baud = result["baud"].as<unsigned>();
  opts.call_timeout = std::chrono::milliseconds(timeout_ms);
  opts.auto_attach = result.count("no-attach") == 0;

  const bool color = result.count("no-color") == 0 && stdout_is_tty();
  const bool monitor = result.count("monitor") > 0;
  const std::vector<std::string> positional = result.count("args") ? result["args"].as<std::vector<std::string>>()
                                                                     : std::vector<std::string>();

  if (!monitor && !positional.empty()) return run_one_shot(port_path, opts.baud, timeout_ms, positional);
  if (monitor) return run_monitor(opts, color);
  return run_repl(opts, color);
}
