/// \file repl.hpp
/// \brief The interactive REPL (docs/PLAN.md component 7): a `replxx`-backed
///   line editor that turns typed lines into `IoWorker` commands and renders
///   `IoWorker`'s output (colour-coded, timestamped) back to the terminal.
///
/// The REPL thread (this class's `run()`) only ever touches `IoWorker`
/// through `IoWorker::submit()` and reads its `attached()`/`connected()`
/// atomics -- never the port. Output arriving from the I/O thread comes
/// through `print_output()`, which is called directly from that thread (see
/// the note on `replxx::Replxx::print()` below) -- so the only state shared
/// between the two threads here is `_filters` (atomics) and `_methods`
/// (mutex-guarded).
#pragma once

#include "io_worker.hpp"
#include "json_value.hpp"

#include <fmt/color.h>
#include <fmt/format.h>
#include <replxx.hxx>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <mutex>
#include <string>
#include <vector>

namespace rpc_repl {

/// `HH:MM:SS.mmm`, local time.
inline std::string format_timestamp() {
  using namespace std::chrono;
  const auto now = system_clock::now();
  const std::time_t t = system_clock::to_time_t(now);
  const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
  std::tm tmv{};
#if defined(_WIN32)
  localtime_s(&tmv, &t);
#else
  localtime_r(&t, &tmv);
#endif
  return fmt::format("{:02d}:{:02d}:{:02d}.{:03d}", tmv.tm_hour, tmv.tm_min, tmv.tm_sec, static_cast<int>(ms.count()));
}

/// Renders one `(Kind, message)` pair (docs/PLAN.md's Output bullets) as a
/// single, optionally coloured, timestamped line, ready to hand to
/// `replxx::Replxx::print()` or plain stdout. `Kind::Log`'s message is
/// `"<level>|<line>"` (see io_worker.hpp's `Kind` doc comment); every other
/// Kind's message is shown as-is.
inline std::string format_output_line(Kind kind, const std::string &msg, bool color) {
  const std::string ts = format_timestamp();
  std::string body;
  fmt::color c = fmt::color::white;

  switch (kind) {
    case Kind::Log: {
      const size_t bar = msg.find('|');
      int level = 1;
      std::string text = msg;
      if (bar != std::string::npos) {
        try {
          level = std::stoi(msg.substr(0, bar));
        } catch (...) {
          level = 1;
        }
        text = msg.substr(bar + 1);
      }
      c = level <= 0 ? fmt::color::gray : level == 1 ? fmt::color::light_sky_blue : level == 2 ? fmt::color::yellow : fmt::color::red;
      body = fmt::format("{} [log {}] {}", ts, level, text);
      break;
    }
    case Kind::Event:
      c = fmt::color::magenta;
      body = fmt::format("{} [evt] {}", ts, msg);
      break;
    case Kind::Text:
      c = fmt::color::gray;
      body = fmt::format("{} [txt] {}", ts, msg);
      break;
    case Kind::Outgoing:
      c = fmt::color::deep_sky_blue;
      body = fmt::format("{} → {}", ts, msg);
      break;
    case Kind::Result:
      c = fmt::color::light_green;
      body = fmt::format("{} ← {}", ts, msg);
      break;
    case Kind::Error:
      c = fmt::color::red;
      body = fmt::format("{} ← error: {}", ts, msg);
      break;
    case Kind::Sys:
      c = fmt::color::gold;
      body = fmt::format("{} [sys] {}", ts, msg);
      break;
  }
  if (!color) return body;
  return fmt::format(fmt::fg(c), "{}", body);
}

/// Portable home-directory lookup (HOME on POSIX, USERPROFILE on Windows),
/// used only to build the history file path.
inline std::string home_dir() {
#if defined(_WIN32)
  const char *h = std::getenv("USERPROFILE");
#else
  const char *h = std::getenv("HOME");
#endif
  return h ? std::string(h) : std::string(".");
}

inline std::string history_file_path() {
#if defined(_WIN32)
  return home_dir() + "\\.rpc_repl_history";
#else
  return home_dir() + "/.rpc_repl_history";
#endif
}

/// Meta-commands (docs/PLAN.md component 7), for tab completion and `.help`.
inline const std::vector<std::string> &meta_commands() {
  static const std::vector<std::string> v = {".help",   ".list",  ".attach", ".detach",    ".notify",
                                              ".text",   ".reset", ".reconnect", ".filter", ".timeout", ".quit"};
  return v;
}

struct ReplOptions {
  bool color = true;
};

/// See this file's top comment. Owns the `replxx::Replxx` instance and
/// drives the read-eval loop; submits everything it parses to an
/// `IoWorker` it doesn't own (constructed by the caller, started/stopped by
/// the caller).
class Repl {
public:
  Repl(IoWorker &worker, ReplOptions opts) : _worker(worker), _opts(opts) {
    _rx.history_load(history_file_path());
    _rx.set_word_break_characters(" \t");
    _rx.set_completion_callback(
        [this](const std::string &input, int &contextLen) { return complete_(input, contextLen); });
    _rx.set_hint_callback([this](const std::string &input, int &contextLen, replxx::Replxx::Color &color) {
      return hint_(input, contextLen, color);
    });
  }

  /// Called from the `IoWorker`'s output-sink lambda (on the I/O thread).
  /// Safe to call while the REPL thread is inside `_rx.input()`: that's
  /// exactly the job `replxx::Replxx::print()` is documented to do
  /// (docs/PLAN.md: "a thread-safe print() that redraws the prompt").
  void print_output(Kind kind, const std::string &msg) {
    if (kind == Kind::Log && !_filters.log.load()) return;
    if (kind == Kind::Event && !_filters.evt.load()) return;
    if (kind == Kind::Text && !_filters.txt.load()) return;
    const std::string line = format_output_line(kind, msg, _opts.color);
    _rx.print("%s\n", line.c_str());
  }

  /// Called from the I/O thread whenever the method list is refreshed
  /// (attach, `.list`, reconnect).
  void set_methods(std::vector<serial_rpc::MethodInfo> methods) {
    std::lock_guard<std::mutex> lock(_methods_mu);
    _methods = std::move(methods);
  }

  /// Runs the read-eval loop until `.quit` or Ctrl-D (`input()` returning
  /// nullptr). Returns an exit code (always 0: the REPL itself doesn't
  /// fail, individual commands report their own errors as output lines).
  int run() {
    while (!_quit) {
      const char *line = _rx.input(make_prompt_());
      if (line == nullptr) break; // Ctrl-D
      const std::string text(line);
      if (text.empty()) continue;
      _rx.history_add(text);
      handle_line_(text);
    }
    _rx.history_save(history_file_path());
    return 0;
  }

private:
  struct Filters {
    std::atomic<bool> log{true};
    std::atomic<bool> evt{true};
    std::atomic<bool> txt{true};
  };

  std::string make_prompt_() const {
    const bool att = _worker.attached();
    const std::string p = att ? "rpc> " : "rpc*> ";
    if (!_opts.color) return p;
    return fmt::format(fmt::fg(att ? fmt::color::light_green : fmt::color::yellow), "{}", p);
  }

  void handle_line_(const std::string &line) {
    // .text takes the raw remainder of the line verbatim -- it is not
    // shell/JSON-tokenized, so brackets, quotes, etc. in it are sent as-is.
    if (line == ".text" || line.rfind(".text ", 0) == 0) {
      _worker.submit(SendTextCmd{line.size() > 6 ? line.substr(6) : std::string()});
      return;
    }

    ParsedCommand parsed;
    try {
      parsed = parse_args(line);
    } catch (const ParseError &e) {
      print_output(Kind::Error, e.what());
      return;
    }
    if (parsed.command.empty()) return;

    if (parsed.command.front() == '.') {
      handle_meta_(parsed);
    } else {
      _worker.submit(CallCmd{parsed.command, std::move(parsed.args)});
    }
  }

  void handle_meta_(ParsedCommand &parsed) {
    const std::string &cmd = parsed.command;
    std::vector<serial_rpc::Value> &args = parsed.args;

    if (cmd == ".help") {
      print_help_();
    } else if (cmd == ".list") {
      _worker.submit(ListCmd{});
    } else if (cmd == ".attach") {
      _worker.submit(AttachCmd{});
    } else if (cmd == ".detach") {
      _worker.submit(DetachCmd{});
    } else if (cmd == ".notify") {
      if (args.empty() || !args[0].is_string()) {
        print_output(Kind::Error, ".notify needs a method name: .notify <method> [args...]");
        return;
      }
      std::string method = args[0].as<std::string>();
      std::vector<serial_rpc::Value> rest(args.begin() + 1, args.end());
      _worker.submit(NotifyCmd{std::move(method), std::move(rest)});
    } else if (cmd == ".reset") {
      const bool hard = !args.empty() && args[0].is_string() && args[0].as<std::string>() == "hard";
      _worker.submit(ResetCmd{hard});
    } else if (cmd == ".reconnect") {
      _worker.submit(ReconnectCmd{});
    } else if (cmd == ".filter") {
      handle_filter_(args);
    } else if (cmd == ".timeout") {
      if (args.empty() || !args[0].is_integer()) {
        print_output(Kind::Error, ".timeout needs a millisecond count: .timeout <ms>");
        return;
      }
      const int64_t ms = args[0].is_int() ? args[0].as<int64_t>() : static_cast<int64_t>(args[0].as<uint64_t>());
      if (ms <= 0) {
        print_output(Kind::Error, ".timeout needs a positive millisecond count");
        return;
      }
      _worker.submit(SetTimeoutCmd{std::chrono::milliseconds(ms)});
    } else if (cmd == ".quit") {
      _quit = true;
    } else {
      print_output(Kind::Error, "unknown meta-command: " + cmd + " (try .help)");
    }
  }

  void handle_filter_(std::vector<serial_rpc::Value> &args) {
    if (args.empty()) {
      print_output(Kind::Sys, fmt::format("filters: log={} evt={} txt={}", _filters.log.load() ? "on" : "off",
                                           _filters.evt.load() ? "on" : "off", _filters.txt.load() ? "on" : "off"));
      return;
    }
    if (!args[0].is_string()) {
      print_output(Kind::Error, ".filter needs [+|-]log|evt|txt");
      return;
    }
    const std::string spec = args[0].as<std::string>();
    bool enable = true;
    size_t pos = 0;
    if (!spec.empty() && (spec.front() == '+' || spec.front() == '-')) {
      enable = spec.front() == '+';
      pos = 1;
    }
    const std::string name = spec.substr(pos);
    std::atomic<bool> *target =
        name == "log" ? &_filters.log : name == "evt" ? &_filters.evt : name == "txt" ? &_filters.txt : nullptr;
    if (!target) {
      print_output(Kind::Error, "unknown filter: " + name + " (expected log, evt or txt)");
      return;
    }
    target->store(enable);
    print_output(Kind::Sys, fmt::format("filter {} {}", name, enable ? "on" : "off"));
  }

  void print_help_() {
    static constexpr const char *kHelp =
        ".help                        show this help\n"
        ".list                        show method names and signatures\n"
        ".attach / .detach            turn the framed protocol on/off\n"
        ".notify <method> [args...]   fire-and-forget call (no reply expected)\n"
        ".text <line>                 send a raw text line, verbatim (not JSON-tokenized)\n"
        ".reset [hard]                pulse DTR, or (hard) a 1200-baud touch reset\n"
        ".reconnect                   close and reopen the port\n"
        ".filter [+|-]log|evt|txt     show/toggle a message filter\n"
        ".timeout <ms>                set the call timeout\n"
        ".quit                        exit (Ctrl-D also works)\n"
        "\n"
        "Call syntax: <method> [args...]. Each arg is parsed as JSON,\n"
        "falling back to a plain string if it isn't valid JSON; quoted\n"
        "args (\"...\" or '...') are always strings. Examples:\n"
        "  set_led 13 true\n"
        "  cfg {\"kp\": 1.5, \"ki\": [0, 1]}\n"
        "  echo \"hi there\"";
    print_output(Kind::Sys, kHelp);
  }

  replxx::Replxx::completions_t complete_(const std::string &input, int &contextLen) {
    replxx::Replxx::completions_t out;
    // Only the first word (the method/meta-command name) is completed.
    if (input.find(' ') != std::string::npos) return out;
    const std::string prefix =
        static_cast<size_t>(contextLen) <= input.size() ? input.substr(input.size() - static_cast<size_t>(contextLen)) : input;

    std::vector<std::string> names = meta_commands();
    {
      std::lock_guard<std::mutex> lock(_methods_mu);
      for (const auto &m : _methods) names.push_back(m.name);
    }
    for (const auto &n : names) {
      if (n.size() >= prefix.size() && n.compare(0, prefix.size(), prefix) == 0) out.emplace_back(n);
    }
    return out;
  }

  replxx::Replxx::hints_t hint_(const std::string &input, int &contextLen, replxx::Replxx::Color &color) {
    (void)contextLen;
    replxx::Replxx::hints_t out;
    const size_t sp = input.find(' ');
    const std::string first = sp == std::string::npos ? input : input.substr(0, sp);
    if (first.empty()) return out;

    std::lock_guard<std::mutex> lock(_methods_mu);
    if (sp == std::string::npos) {
      const serial_rpc::MethodInfo *match = nullptr;
      int count = 0;
      for (const auto &m : _methods) {
        if (m.name.size() >= first.size() && m.name.compare(0, first.size(), first) == 0) {
          match = &m;
          ++count;
        }
      }
      if (count == 1 && match->name != first) {
        std::string h = match->name.substr(first.size());
        if (!match->signature.empty()) h += "  " + match->signature;
        out.push_back(h);
        color = replxx::Replxx::Color::GRAY;
      }
    } else {
      for (const auto &m : _methods) {
        if (m.name == first && !m.signature.empty()) {
          out.push_back("  " + m.signature);
          color = replxx::Replxx::Color::GRAY;
          break;
        }
      }
    }
    return out;
  }

  IoWorker &_worker;
  ReplOptions _opts;
  replxx::Replxx _rx;
  bool _quit = false;
  Filters _filters;

  std::mutex _methods_mu;
  std::vector<serial_rpc::MethodInfo> _methods;
};

} // namespace rpc_repl
