// SPDX-License-Identifier: Apache-2.0
/*
  ____            _       _ ____            _
 / ___|  ___ _ __(_) __ _| |  _ \ ___  _ __| |_
 \___ \ / _ \ '__| |/ _` | | |_) / _ \| '__| __|
  ___) |  __/ |  | | (_| | |  __/ (_) | |  | |_
 |____/ \___|_|  |_|\__,_|_|_|   \___/|_|   \__|

Serial port interface for Linux, macOS and Windows, C++20, header-only.
The class is a thin, RAII wrapper over the native port handle: it never
busy-waits, it buffers input internally, and it restores the original port
settings on close. Line framing, timeouts and buffering are implemented once,
in the shared layer below; everything that talks to the operating system
lives in the platform layer, implemented twice: termios plus poll(2) on
POSIX, DCB plus COMMTIMEOUTS on Windows. Every definition here is `inline`
(or has internal/merged linkage) so that this single header can be included
from more than one translation unit without violating the One Definition
Rule; platform-only helpers live in namespace serialport_detail so they
don't leak into code that includes this header.
*/

#ifndef SERIALPORT_HPP
#define SERIALPORT_HPP

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#if defined(_WIN32)

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <cctype>
#include <cstdlib>

#else

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <filesystem>

// Non-standard baud rates are supported on macOS only, through IOSSIOSPEED.
#if defined(__APPLE__) && __has_include(<IOKit/serial/ioss.h>)
#include <IOKit/serial/ioss.h>
#define SERIALPORT_HAS_CUSTOM_SPEED 1
#endif

#ifndef _POSIX_VDISABLE
#define _POSIX_VDISABLE 0
#endif

// CRTSCTS is not in POSIX: Linux has it, macOS spells it with two flags.
#ifndef CRTSCTS
#if defined(CCTS_OFLOW) && defined(CRTS_IFLOW)
#define CRTSCTS (CCTS_OFLOW | CRTS_IFLOW)
#else
#define CRTSCTS 0
#endif
#endif

#endif /* _WIN32 */

/** Class to interface serial ports under Linux, macOS and Windows.
 *
 * The port is opened by the constructor and closed by the destructor. Objects
 * are movable but not copyable, so that a handle is never closed twice. A
 * single port must not be used concurrently from more than one thread.
 *
 * All the I/O methods honour the configured timeout() and report failures via
 * a negative return value; the reason of the failure is available through
 * last_error(), while timed_out() tells whether a short read was caused by the
 * timeout expiring. The setup and control methods (the constructors,
 * set_baud_rate(), the flush family, drain() and the modem line accessors)
 * throw std::system_error or std::invalid_argument instead.
 */
class SerialPort {
public:
  /** Steady clock used for all timeouts. */
  using Clock = std::chrono::steady_clock;
  /** Convenience alias for timeout arguments. */
  using Milliseconds = std::chrono::milliseconds;

#if defined(_WIN32)
  /** Native port handle: a Win32 HANDLE, declared as void * so that this
   * header does not need to include windows.h. */
  using NativeHandle = void *;
  /** Value of native_handle() when the port is closed. */
  static constexpr NativeHandle invalid_handle = nullptr;
#else
  /** Native port handle: a file descriptor. */
  using NativeHandle = int;
  /** Value of native_handle() when the port is closed. */
  static constexpr NativeHandle invalid_handle = -1;
#endif

  /** Parity generation/checking mode. */
  enum class Parity { none, even, odd };

  /** Flow control mode. */
  enum class FlowControl { none, software, hardware };

  /** Timeout value meaning "block until the operation completes". */
  static constexpr Milliseconds no_timeout{-1};

  /** Full port configuration, designed for designated initializers:
   * @code
   * SerialPort port("/dev/ttyUSB0", {.baud_rate = 115200, .timeout = 500ms});
   * @endcode
   */
  struct Config {
    /** Baud rate, e.g. 9600 or 115200. Linux only accepts the rates listed
     * in termios.h, macOS and Windows accept any rate the driver supports. */
    unsigned baud_rate = 57600;
    /** Bits per character (5 to 8). */
    unsigned data_bits = 8;
    /** Number of stop bits (1 or 2). */
    unsigned stop_bits = 1;
    /** Parity mode. */
    Parity parity = Parity::none;
    /** Flow control mode. */
    FlowControl flow_control = FlowControl::none;
    /** Line-oriented input processing (ICANON). POSIX only: read_line()
     * does its own framing, so this rarely needs to be changed. */
    bool canonical_mode = false;
    /** Lower the modem lines when the port is closed (HUPCL). POSIX only. */
    bool hangup_on_close = true;
    /** Refuse any further open of the same port (TIOCEXCL). POSIX only:
     * Windows always opens serial ports exclusively. */
    bool exclusive = true;
    /** Timeout of every read and write, no_timeout to block. */
    Milliseconds timeout = no_timeout;
    /** Size of the internal input buffer, at least 64 bytes. */
    std::size_t buffer_size = 1024;
    /** Longest line accepted by read_line(). */
    std::size_t max_line_length = 65536;
  };

  /** Open a serial port.
   *  @param[in] port Path to the serial port, e.g. "/dev/ttyUSB0".
   *  @param[in] baud_rate Serial baud rate configuration.
   *  @param[in] stop_bits Number of stop bits (1 or 2).
   *  @param[in] canonical_mode Enable canonical (line oriented) input.
   *  @throws std::invalid_argument on an unsupported parameter.
   *  @throws std::system_error if the port cannot be opened or configured.
   */
  explicit SerialPort(std::string_view port, unsigned baud_rate = 57600,
                      unsigned stop_bits = 1, bool canonical_mode = false);

  /** Open a serial port with a full configuration.
   *  @param[in] port Path to the serial port.
   *  @param[in] config Port configuration.
   *  @throws std::invalid_argument on an unsupported parameter.
   *  @throws std::system_error if the port cannot be opened or configured.
   */
  SerialPort(std::string_view port, const Config &config);

  /** Restore the original port settings and close the descriptor. */
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort &operator=(const SerialPort &) = delete;
  SerialPort(SerialPort &&other) noexcept;
  SerialPort &operator=(SerialPort &&other) noexcept;

  /* WRITING ****************************************************************/

  /** Write a buffer, retrying until everything is written or the timeout
   * expires.
   * @param[in] buf Output buffer.
   * @param[in] n_bytes Number of bytes to be written.
   * @return Number of bytes written, negative on error.
   */
  int write(const char *buf, std::size_t n_bytes);

  /** Write a string (also accepts a std::string or a C string).
   * @param[in] data Text to be written.
   * @return Number of bytes written, negative on error.
   */
  int write(std::string_view data);

  /** Write raw bytes.
   * @param[in] data Bytes to be written.
   * @return Number of bytes written, negative on error.
   */
  int write(std::span<const std::byte> data);

  /** Write a string followed by a line terminator.
   * @param[in] data Text to be written.
   * @param[in] eol Line terminator appended to @p data.
   * @return Number of bytes written, negative on error.
   */
  int write_line(std::string_view data, std::string_view eol = "\n");

  /* READING ****************************************************************/

  /** Read exactly n_bytes into a buffer, unless the timeout expires first.
   * @param[out] buf Buffer to be filled.
   * @param[in] n_bytes Number of bytes to be read.
   * @return Number of bytes read (less than @p n_bytes on timeout), negative
   *         on error.
   */
  int read(char *buf, std::size_t n_bytes);

  /** Read exactly buf.size() bytes, unless the timeout expires first.
   * @param[out] buf Buffer to be filled.
   * @return Number of bytes read, negative on error.
   */
  int read(std::span<char> buf);

  /** Read exactly buf.size() bytes, unless the timeout expires first.
   * @param[out] buf Buffer to be filled.
   * @return Number of bytes read, negative on error.
   */
  int read(std::span<std::byte> buf);

  /** Read whatever is available, waiting for at least one byte.
   * @param[out] buf Buffer to be filled.
   * @param[in] n_max Capacity of @p buf.
   * @return Number of bytes read (0 on timeout), negative on error.
   */
  int read_some(char *buf, std::size_t n_max);

  /** Read a line terminated by a newline character (CR, LF or CRLF).
   * The terminator is consumed but not stored. A line longer than
   * @p n_max - 1 is truncated (the rest is discarded, so that the framing of
   * the following lines is preserved) and last_error() is set to EMSGSIZE.
   * @param[out] line Line read, NULL-terminated.
   * @param[in] n_max Capacity of line buffer inclusive NULL-termination.
   * @return Number of bytes read (without NULL-termination), negative on
   *         error.
   */
  int read_line(char *line, std::size_t n_max);

  /** Read a line terminated by a newline character (CR, LF or CRLF).
   * @p line is cleared first; the terminator is consumed but not stored.
   * @param[out] line Line read as a std::string.
   * @return Number of bytes read, negative on error.
   */
  int read_line(std::string &line);

  /** Read a line terminated by a newline character (CR, LF or CRLF).
   * @return The line read, or std::nullopt on error or timeout.
   */
  [[nodiscard]] std::optional<std::string> read_line();

  /** Alias of read_line(char *, size_t), kept for backward compatibility. */
  int readLine(char *line, std::size_t n_max) { return read_line(line, n_max); }

  /** Alias of read_line(std::string &), kept for backward compatibility. */
  int readLine(std::string &line) { return read_line(line); }

  /* PORT STATE *************************************************************/

  /** @return true if the port is open. */
  [[nodiscard]] bool is_open() const noexcept {
    return _handle != invalid_handle;
  }

  /** Drain, restore the original settings and close the port. Idempotent. */
  void close() noexcept;

  /** @return The native handle, invalid_handle if the port is closed. */
  [[nodiscard]] NativeHandle native_handle() const noexcept { return _handle; }

#if !defined(_WIN32)
  /** @return The underlying file descriptor, -1 if the port is closed.
   * Available on POSIX systems only, use native_handle() for portable code. */
  [[nodiscard]] int fd() const noexcept { return _handle; }
#endif

  /** @return The path of the port. */
  [[nodiscard]] const std::string &port() const noexcept { return _port; }

  /** @return The current configuration. */
  [[nodiscard]] const Config &config() const noexcept { return _config; }

  /** @return The current read/write timeout. */
  [[nodiscard]] Milliseconds timeout() const noexcept {
    return _config.timeout;
  }

  /** Set the read/write timeout (no_timeout to block indefinitely).
   * @param[in] value New timeout.
   */
  void set_timeout(Milliseconds value) noexcept { _config.timeout = value; }

  /** @return The current baud rate. */
  [[nodiscard]] unsigned baud_rate() const noexcept {
    return _config.baud_rate;
  }

  /** Change the baud rate on an open port.
   * @param[in] value New baud rate.
   * @throws std::invalid_argument on an unsupported baud rate.
   * @throws std::system_error if the port cannot be reconfigured.
   */
  void set_baud_rate(unsigned value);

  /** @return true if the last I/O operation was cut short by the timeout. */
  [[nodiscard]] bool timed_out() const noexcept { return _timed_out; }

  /** @return The error reported by the last failed operation. */
  [[nodiscard]] std::error_code last_error() const noexcept {
    return _last_error;
  }

  /** @return Number of bytes available for reading without blocking. */
  [[nodiscard]] std::size_t available() const;

  /** Discard all the data received but not read yet. */
  void flush_input();

  /** Discard all the data written but not transmitted yet. */
  void flush_output();

  /** Discard both pending input and pending output. */
  void flush();

  /** Block until all the pending output has been transmitted. */
  void drain();

  /* MODEM LINES ************************************************************/

  /** Set or clear the DTR line (toggling it resets most Arduino boards).
   * @param[in] on New state of the line.
   */
  void set_dtr(bool on);

  /** Set or clear the RTS line.
   * @param[in] on New state of the line.
   */
  void set_rts(bool on);

  /** @return The state of the CTS input line. */
  [[nodiscard]] bool cts() const;

  /** @return The state of the DSR input line. */
  [[nodiscard]] bool dsr() const;

  /** @return The state of the DCD (carrier detect) input line. */
  [[nodiscard]] bool dcd() const;

  /** Transmit a stream of zero bits.
   * @param[in] duration On POSIX an implementation defined duration, on
   *            Windows a number of milliseconds; 0 means about 0.25 s on both.
   */
  void send_break(int duration = 0);

  /** List the serial ports that look usable on this machine.
   * @return Paths of the candidate ports, sorted, possibly empty.
   */
  [[nodiscard]] static std::vector<std::string> available_ports();

private:
  /** Absolute instant after which an operation gives up, or nullopt if it
   * must block indefinitely. */
  using Deadline = std::optional<Clock::time_point>;

  /** One of the modem control lines. */
  enum class ModemLine { dtr, rts, cts, dsr, dcd };

  /** Platform specific state, kept out of the header. */
  struct State;

  /* Platform layer: one implementation per operating system. Each of these
     either throws (setup and control) or returns the number of bytes moved,
     0 on timeout and -1 on error after calling fail_native(). */
  void open_native(const Config &config);
  void close_native() noexcept;
  void apply_config(const Config &config);
  int read_native(char *buf, std::size_t n_max, const Deadline &deadline);
  int write_native(const char *buf, std::size_t n_bytes,
                   const Deadline &deadline);
  [[nodiscard]] std::size_t available_native() const;
  void flush_native(bool input, bool output);
  void drain_native();
  void modem_set_native(ModemLine line, bool on);
  [[nodiscard]] bool modem_get_native(ModemLine line) const;
  void break_native(int duration);

  /* Shared layer. */
  int fill_buffer(const Deadline &deadline);
  std::size_t take_buffered(char *dst, std::size_t n_max) noexcept;
  int fail(std::errc code) const noexcept;
  int fail_native(int error_number) const noexcept;
  void begin_operation() noexcept;

  [[nodiscard]] std::size_t buffered() const noexcept {
    return _rx_tail - _rx_head;
  }

  std::string _port;
  Config _config{};
  std::unique_ptr<State> _state;
  std::vector<char> _rx_buf;
  std::string _line_buf;
  std::size_t _rx_head = 0;
  std::size_t _rx_tail = 0;
  NativeHandle _handle = invalid_handle;
  bool _pending_lf = false;
  bool _timed_out = false;
  mutable std::error_code _last_error{};
};

/** Platform specific state, defined right after the class so that neither
 * windows.h nor the termios macros need to be visible at the point where
 * SerialPort itself is declared. */
#if defined(_WIN32)
struct SerialPort::State {
  DCB original{};                   /**< Settings to be restored on close */
  COMMTIMEOUTS original_timeouts{}; /**< Timeouts to be restored on close */
  bool restore = false;             /**< True once original has been read */
};
#else
struct SerialPort::State {
  termios original{};   /**< Settings to be restored on close */
  bool restore = false; /**< True once original has been read */
};
#endif

/** File-local helpers, kept in a named namespace (rather than anonymous)
 * because this header can be included by more than one translation unit;
 * everything here is `inline` so that repeated inclusion never violates the
 * One Definition Rule. */
namespace serialport_detail {

using Deadline = std::optional<SerialPort::Clock::time_point>;

/** @return The error code of the last failed system call. */
inline int last_native_error() {
#if defined(_WIN32)
  return static_cast<int>(::GetLastError());
#else
  return errno;
#endif
}

/** Throw a std::system_error carrying a native error code. */
[[noreturn]] inline void throw_native(int error_number, const std::string &what) {
  throw std::system_error(error_number, std::system_category(), what);
}

/** Throw a std::system_error carrying a portable error condition. */
[[noreturn]] inline void throw_generic(std::errc code, const std::string &what) {
  throw std::system_error(std::make_error_code(code), what);
}

/** Turn a timeout into an absolute deadline.
 * @param[in] timeout Negative for "block indefinitely".
 * @return The deadline, or std::nullopt if the operation must block.
 */
inline Deadline make_deadline(SerialPort::Milliseconds timeout) {
  if (timeout < SerialPort::Milliseconds::zero()) {
    return std::nullopt;
  }
  return SerialPort::Clock::now() + timeout;
}

/** Milliseconds left before a deadline.
 * @return -1 to block forever, 0 if the deadline has already expired.
 */
inline int remaining_ms(const Deadline &deadline) {
  if (!deadline) {
    return -1;
  }
  const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
      *deadline - SerialPort::Clock::now());
  if (left.count() <= 0) {
    return 0;
  }
  constexpr auto max = static_cast<long long>(std::numeric_limits<int>::max());
  return static_cast<int>(std::min<long long>(left.count(), max));
}

/** Saturating conversion of a byte count to the int returned by the API. */
inline int clamp_to_int(std::size_t count) noexcept {
  constexpr auto max =
      static_cast<std::size_t>(std::numeric_limits<int>::max());
  return static_cast<int>(std::min(count, max));
}

/* PLATFORM HELPERS *********************************************************/

#if defined(_WIN32)

/** Any rate the driver accepts is legal on Windows. */
inline bool baud_rate_supported(unsigned rate) { return rate > 0; }

/** Full device path of a port: COM10 and above are only reachable through
 * the \\.\ namespace, and a path that is already absolute is left alone.
 * @param[in] port Port name as given by the caller.
 * @return The name to be handed to CreateFile().
 */
inline std::string device_path(const std::string &port) {
  if (port.starts_with("\\\\")) {
    return port;
  }
  if (port.size() >= 3 && (port[0] == 'C' || port[0] == 'c') &&
      (port[1] == 'O' || port[1] == 'o') &&
      (port[2] == 'M' || port[2] == 'm')) {
    return "\\\\.\\" + port;
  }
  return port;
}

#else

/** Association between a numeric baud rate and its termios code. */
struct BaudEntry {
  unsigned rate;
  speed_t code;
};

inline constexpr BaudEntry BaudRates[] = {
    {50, B50},           {75, B75},         {110, B110},     {134, B134},
    {150, B150},         {200, B200},       {300, B300},     {600, B600},
    {1200, B1200},       {1800, B1800},     {2400, B2400},   {4800, B4800},
    {9600, B9600},       {19200, B19200},   {38400, B38400}, {57600, B57600},
    {115200, B115200},   {230400, B230400},
#ifdef B460800
    {460800, B460800},
#endif
#ifdef B500000
    {500000, B500000},
#endif
#ifdef B576000
    {576000, B576000},
#endif
#ifdef B921600
    {921600, B921600},
#endif
#ifdef B1000000
    {1000000, B1000000},
#endif
#ifdef B1152000
    {1152000, B1152000},
#endif
#ifdef B1500000
    {1500000, B1500000},
#endif
#ifdef B2000000
    {2000000, B2000000},
#endif
#ifdef B2500000
    {2500000, B2500000},
#endif
#ifdef B3000000
    {3000000, B3000000},
#endif
#ifdef B3500000
    {3500000, B3500000},
#endif
#ifdef B4000000
    {4000000, B4000000},
#endif
};

/** Look up the termios code of a baud rate.
 * @param[in] rate Baud rate.
 * @return The termios code, or std::nullopt if the rate is not standard.
 */
inline std::optional<speed_t> baud_code(unsigned rate) {
  const auto it = std::ranges::find(BaudRates, rate, &BaudEntry::rate);
  if (it == std::ranges::end(BaudRates)) {
    return std::nullopt;
  }
  return it->code;
}

/** Only the rates listed in termios.h are legal, unless the system can set
 * an arbitrary speed after the fact. */
inline bool baud_rate_supported(unsigned rate) {
#ifdef SERIALPORT_HAS_CUSTOM_SPEED
  return rate > 0;
#else
  return baud_code(rate).has_value();
#endif
}

/** Wait until a descriptor is ready, without ever busy-waiting.
 * @param[in] fd Descriptor to wait on.
 * @param[in] for_write Wait for writability instead of readability.
 * @param[in] deadline When to give up.
 * @return 1 when ready, 0 on timeout, -1 on error with errno set.
 */
inline int wait_ready(int fd, bool for_write, const Deadline &deadline) {
  const short events = static_cast<short>(for_write ? POLLOUT : POLLIN);
  pollfd descriptor{.fd = fd, .events = events, .revents = 0};
  while (true) {
    const int ret = ::poll(&descriptor, 1, remaining_ms(deadline));
    if (ret > 0) {
      if ((descriptor.revents & events) != 0) {
        return 1;
      }
      // POLLERR or POLLHUP with no data left: the device is gone.
      errno = (descriptor.revents & POLLNVAL) != 0 ? EBADF : EIO;
      return -1;
    }
    if (ret == 0) {
      return 0;
    }
    if (errno != EINTR) {
      return -1;
    }
  }
}

#endif /* _WIN32 */

/** Validate a configuration before touching the hardware.
 * @throws std::invalid_argument on any unsupported value.
 */
inline void validate(const SerialPort::Config &config, std::string_view port) {
  if (port.empty()) {
    throw std::invalid_argument("SerialPort: empty port name");
  }
  const std::string where = " for " + std::string(port);
  if (config.data_bits < 5 || config.data_bits > 8) {
    throw std::invalid_argument("SerialPort: data bits must be 5 to 8" + where);
  }
  if (config.stop_bits != 1 && config.stop_bits != 2) {
    throw std::invalid_argument("SerialPort: stop bits must be 1 or 2" + where);
  }
  if (config.max_line_length == 0) {
    throw std::invalid_argument("SerialPort: zero max line length" + where);
  }
  if (!baud_rate_supported(config.baud_rate)) {
    throw std::invalid_argument("SerialPort: unsupported baud rate " +
                                std::to_string(config.baud_rate) + where);
  }
}

} // namespace serialport_detail

/* LIFECYCLE ****************************************************************/

inline SerialPort::SerialPort(std::string_view port, unsigned baud_rate,
                              unsigned stop_bits, bool canonical_mode)
    : SerialPort(port, Config{.baud_rate = baud_rate,
                              .stop_bits = stop_bits,
                              .canonical_mode = canonical_mode}) {}

inline SerialPort::SerialPort(std::string_view port, const Config &config)
    : _port(port), _config(config), _state(std::make_unique<State>()),
      _rx_buf(std::max<std::size_t>(config.buffer_size, 64)) {
  serialport_detail::validate(_config, _port);
  try {
    open_native(_config);
  } catch (...) {
    close();
    throw;
  }
}

inline SerialPort::SerialPort(SerialPort &&other) noexcept
    : _port(std::move(other._port)), _config(other._config),
      _state(std::move(other._state)), _rx_buf(std::move(other._rx_buf)),
      _line_buf(std::move(other._line_buf)), _rx_head(other._rx_head),
      _rx_tail(other._rx_tail),
      _handle(std::exchange(other._handle, invalid_handle)),
      _pending_lf(other._pending_lf), _timed_out(other._timed_out),
      _last_error(other._last_error) {
  other._rx_head = 0;
  other._rx_tail = 0;
  other._pending_lf = false;
}

inline SerialPort &SerialPort::operator=(SerialPort &&other) noexcept {
  if (this != &other) {
    close();
    _port = std::move(other._port);
    _config = other._config;
    _state = std::move(other._state);
    _rx_buf = std::move(other._rx_buf);
    _line_buf = std::move(other._line_buf);
    _rx_head = std::exchange(other._rx_head, 0);
    _rx_tail = std::exchange(other._rx_tail, 0);
    _handle = std::exchange(other._handle, invalid_handle);
    _pending_lf = std::exchange(other._pending_lf, false);
    _timed_out = other._timed_out;
    _last_error = other._last_error;
  }
  return *this;
}

inline SerialPort::~SerialPort() { close(); }

inline void SerialPort::close() noexcept {
  if (is_open()) {
    close_native();
    _handle = invalid_handle;
  }
  if (_state) {
    _state->restore = false;
  }
  _rx_head = 0;
  _rx_tail = 0;
  _pending_lf = false;
}

/* PLATFORM LAYER: WINDOWS **************************************************/

#if defined(_WIN32)

inline void SerialPort::open_native(const Config &config) {
  // A share mode of zero is what makes the port exclusive, which is the only
  // mode Windows offers for serial devices.
  const HANDLE handle = ::CreateFileA(
      serialport_detail::device_path(_port).c_str(), GENERIC_READ | GENERIC_WRITE,
      0, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (handle == INVALID_HANDLE_VALUE) {
    serialport_detail::throw_native(serialport_detail::last_native_error(),
                                    "SerialPort: cannot open " + _port);
  }
  _handle = handle;

  _state->original.DCBlength = sizeof(DCB);
  if (!::GetCommState(handle, &_state->original)) {
    serialport_detail::throw_native(
        serialport_detail::last_native_error(),
        "SerialPort: " + _port + " is not a serial device");
  }
  if (!::GetCommTimeouts(handle, &_state->original_timeouts)) {
    serialport_detail::throw_native(serialport_detail::last_native_error(),
                                    "SerialPort: cannot read the timeouts of " +
                                        _port);
  }
  _state->restore = true;

  apply_config(config);
  ::PurgeComm(handle,
              PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
}

inline void SerialPort::close_native() noexcept {
  const HANDLE handle = static_cast<HANDLE>(_handle);
  // Let the pending output go out, then put the port back the way it was.
  ::FlushFileBuffers(handle);
  if (_state && _state->restore) {
    ::SetCommState(handle, &_state->original);
    ::SetCommTimeouts(handle, &_state->original_timeouts);
  }
  ::PurgeComm(handle, PURGE_RXCLEAR);
  ::CloseHandle(handle);
}

inline void SerialPort::apply_config(const Config &config) {
  const HANDLE handle = static_cast<HANDLE>(_handle);
  DCB dcb = _state->original;
  dcb.DCBlength = sizeof(DCB);
  dcb.BaudRate = config.baud_rate;
  dcb.ByteSize = static_cast<BYTE>(config.data_bits);
  dcb.StopBits =
      static_cast<BYTE>(config.stop_bits == 2 ? TWOSTOPBITS : ONESTOPBIT);
  switch (config.parity) {
  case Parity::even:
    dcb.Parity = EVENPARITY;
    break;
  case Parity::odd:
    dcb.Parity = ODDPARITY;
    break;
  case Parity::none:
  default:
    dcb.Parity = NOPARITY;
    break;
  }

  // Raw, 8 bit clean transfers: parity is generated but input parity errors
  // are not flagged (as on POSIX without INPCK), no character is substituted
  // or stripped, and an error never stops the I/O until it is acknowledged.
  dcb.fBinary = TRUE;
  dcb.fParity = FALSE;
  dcb.fErrorChar = FALSE;
  dcb.ErrorChar = 0;
  dcb.fNull = FALSE;
  dcb.fAbortOnError = FALSE;
  dcb.fDsrSensitivity = FALSE;
  dcb.fOutxDsrFlow = FALSE;
  dcb.fTXContinueOnXoff = TRUE;

  // DTR is asserted on open, as a POSIX open does.
  dcb.fDtrControl = DTR_CONTROL_ENABLE;
  const bool hardware = config.flow_control == FlowControl::hardware;
  const bool software = config.flow_control == FlowControl::software;
  dcb.fOutxCtsFlow = hardware ? TRUE : FALSE;
  dcb.fRtsControl =
      static_cast<DWORD>(hardware ? RTS_CONTROL_HANDSHAKE : RTS_CONTROL_ENABLE);
  dcb.fOutX = software ? TRUE : FALSE;
  dcb.fInX = software ? TRUE : FALSE;
  dcb.XonChar = 0x11;  // DC1
  dcb.XoffChar = 0x13; // DC3
  dcb.XonLim = 128;
  dcb.XoffLim = 128;

  if (!::SetCommState(handle, &dcb)) {
    serialport_detail::throw_native(serialport_detail::last_native_error(),
                                    "SerialPort: cannot configure " + _port);
  }

  // SetCommState() can apply part of a request: read the settings back to
  // make sure the line is really what was asked for.
  DCB applied{};
  applied.DCBlength = sizeof(DCB);
  if (!::GetCommState(handle, &applied)) {
    serialport_detail::throw_native(
        serialport_detail::last_native_error(),
        "SerialPort: cannot read back the settings of " + _port);
  }
  if (applied.ByteSize != dcb.ByteSize || applied.StopBits != dcb.StopBits ||
      applied.Parity != dcb.Parity) {
    serialport_detail::throw_generic(std::errc::invalid_argument,
                                     "SerialPort: " + _port +
                                         " rejected the requested frame format");
  }
  if (applied.BaudRate != dcb.BaudRate) {
    serialport_detail::throw_generic(
        std::errc::invalid_argument,
        "SerialPort: " + _port + " rejected the baud rate " +
            std::to_string(config.baud_rate));
  }
}

inline int SerialPort::read_native(char *buf, std::size_t n_max,
                                   const Deadline &deadline) {
  const HANDLE handle = static_cast<HANDLE>(_handle);
  const int left = serialport_detail::remaining_ms(deadline);

  // ReadIntervalTimeout of MAXDWORD asks the driver to return as soon as
  // anything has arrived; the total timeout then bounds the wait.
  COMMTIMEOUTS timeouts{};
  timeouts.ReadIntervalTimeout = MAXDWORD;
  if (left < 0) {
    timeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = MAXDWORD - 1; // about 49 days
  } else if (left > 0) {
    timeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = static_cast<DWORD>(left);
  } else {
    // Expired: take whatever the driver already has and return.
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
  }
  if (!::SetCommTimeouts(handle, &timeouts)) {
    return fail_native(serialport_detail::last_native_error());
  }

  DWORD got = 0;
  const DWORD want =
      static_cast<DWORD>(std::min<std::size_t>(n_max, MAXDWORD - 1));
  if (!::ReadFile(handle, buf, want, &got, nullptr)) {
    return fail_native(serialport_detail::last_native_error());
  }
  return serialport_detail::clamp_to_int(got); // zero means the timeout expired
}

inline int SerialPort::write_native(const char *buf, std::size_t n_bytes,
                                    const Deadline &deadline) {
  const HANDLE handle = static_cast<HANDLE>(_handle);
  const int left = serialport_detail::remaining_ms(deadline);

  COMMTIMEOUTS timeouts{};
  timeouts.ReadIntervalTimeout = MAXDWORD; // leave reads non-blocking
  timeouts.WriteTotalTimeoutMultiplier = 0;
  // Zero means "no timeout" for a write, so an expired deadline still gets
  // the shortest wait Windows can express.
  timeouts.WriteTotalTimeoutConstant =
      left < 0 ? 0 : static_cast<DWORD>(std::max(left, 1));
  if (!::SetCommTimeouts(handle, &timeouts)) {
    return fail_native(serialport_detail::last_native_error());
  }

  DWORD written = 0;
  const DWORD want =
      static_cast<DWORD>(std::min<std::size_t>(n_bytes, MAXDWORD - 1));
  if (!::WriteFile(handle, buf, want, &written, nullptr)) {
    return fail_native(serialport_detail::last_native_error());
  }
  return serialport_detail::clamp_to_int(written); // zero means the timeout expired
}

inline std::size_t SerialPort::available_native() const {
  DWORD errors = 0;
  COMSTAT status{};
  if (!::ClearCommError(static_cast<HANDLE>(_handle), &errors, &status)) {
    fail_native(serialport_detail::last_native_error());
    return 0;
  }
  return static_cast<std::size_t>(status.cbInQue);
}

inline void SerialPort::flush_native(bool input, bool output) {
  DWORD flags = 0;
  if (input) {
    flags |= PURGE_RXCLEAR | PURGE_RXABORT;
  }
  if (output) {
    flags |= PURGE_TXCLEAR | PURGE_TXABORT;
  }
  if (flags != 0 && !::PurgeComm(static_cast<HANDLE>(_handle), flags)) {
    serialport_detail::throw_native(serialport_detail::last_native_error(),
                                    "SerialPort: cannot flush " + _port);
  }
}

inline void SerialPort::drain_native() {
  if (!::FlushFileBuffers(static_cast<HANDLE>(_handle))) {
    serialport_detail::throw_native(serialport_detail::last_native_error(),
                                    "SerialPort: cannot drain " + _port);
  }
}

inline void SerialPort::modem_set_native(ModemLine line, bool on) {
  DWORD function = 0;
  switch (line) {
  case ModemLine::dtr:
    function = on ? SETDTR : CLRDTR;
    break;
  case ModemLine::rts:
    function = on ? SETRTS : CLRRTS;
    break;
  default:
    serialport_detail::throw_generic(std::errc::invalid_argument,
                                     "SerialPort: not an output line on " +
                                         _port);
  }
  if (!::EscapeCommFunction(static_cast<HANDLE>(_handle), function)) {
    serialport_detail::throw_native(
        serialport_detail::last_native_error(),
        "SerialPort: cannot set the modem lines of " + _port);
  }
}

inline bool SerialPort::modem_get_native(ModemLine line) const {
  DWORD status = 0;
  if (!::GetCommModemStatus(static_cast<HANDLE>(_handle), &status)) {
    serialport_detail::throw_native(
        serialport_detail::last_native_error(),
        "SerialPort: cannot read the modem lines of " + _port);
  }
  switch (line) {
  case ModemLine::cts:
    return (status & MS_CTS_ON) != 0;
  case ModemLine::dsr:
    return (status & MS_DSR_ON) != 0;
  case ModemLine::dcd:
    return (status & MS_RLSD_ON) != 0;
  default:
    serialport_detail::throw_generic(std::errc::invalid_argument,
                                     "SerialPort: not an input line on " +
                                         _port);
  }
}

inline void SerialPort::break_native(int duration) {
  const HANDLE handle = static_cast<HANDLE>(_handle);
  if (!::SetCommBreak(handle)) {
    serialport_detail::throw_native(serialport_detail::last_native_error(),
                                    "SerialPort: cannot send a break on " +
                                        _port);
  }
  ::Sleep(static_cast<DWORD>(duration > 0 ? duration : 250));
  if (!::ClearCommBreak(handle)) {
    serialport_detail::throw_native(serialport_detail::last_native_error(),
                                    "SerialPort: cannot end the break on " +
                                        _port);
  }
}

inline std::vector<std::string> SerialPort::available_ports() {
  std::vector<std::string> ports;
  std::vector<char> names(1u << 16);
  DWORD length = 0;
  while (true) {
    length = ::QueryDosDeviceA(nullptr, names.data(),
                               static_cast<DWORD>(names.size()));
    if (length != 0) {
      break;
    }
    if (::GetLastError() != ERROR_INSUFFICIENT_BUFFER ||
        names.size() >= (1u << 22)) {
      return ports;
    }
    names.resize(names.size() * 2);
  }

  // The result is a list of NUL terminated names, ended by an empty one.
  const char *const end = names.data() + length;
  for (const char *it = names.data(); it < end && *it != '\0';
       it += std::strlen(it) + 1) {
    const std::string_view name{it};
    if (name.size() < 4 || (name[0] != 'C' && name[0] != 'c') ||
        (name[1] != 'O' && name[1] != 'o') ||
        (name[2] != 'M' && name[2] != 'm')) {
      continue;
    }
    if (std::all_of(name.begin() + 3, name.end(), [](char c) {
          return std::isdigit(static_cast<unsigned char>(c)) != 0;
        })) {
      ports.emplace_back(name);
    }
  }

  // COM2 must come before COM10, so sort on the number, not on the text.
  std::ranges::sort(ports, [](const std::string &a, const std::string &b) {
    return std::strtoul(a.c_str() + 3, nullptr, 10) <
           std::strtoul(b.c_str() + 3, nullptr, 10);
  });
  return ports;
}

/* PLATFORM LAYER: POSIX ****************************************************/

#else

inline void SerialPort::open_native(const Config &config) {
  // O_NONBLOCK: never wait for the carrier, all the timing is done by poll(2).
  // O_NOCTTY: the port must not become the controlling terminal.
  // O_CLOEXEC: the descriptor must not leak into child processes.
  const int fd =
      ::open(_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    serialport_detail::throw_native(serialport_detail::last_native_error(),
                                    "SerialPort: cannot open " + _port);
  }
  _handle = fd;

  if (config.exclusive && ::ioctl(fd, TIOCEXCL) < 0) {
    serialport_detail::throw_native(
        serialport_detail::last_native_error(),
        "SerialPort: cannot get exclusive access to " + _port);
  }
  if (::tcgetattr(fd, &_state->original) < 0) {
    serialport_detail::throw_native(
        serialport_detail::last_native_error(),
        "SerialPort: " + _port + " is not a serial device");
  }
  _state->restore = true;

  apply_config(config);
  ::tcflush(fd, TCIOFLUSH);
}

inline void SerialPort::close_native() noexcept {
  // Let the pending output go out, then put the port back the way it was.
  while (::tcdrain(_handle) < 0 && errno == EINTR) {
  }
  if (_state && _state->restore) {
    ::tcsetattr(_handle, TCSANOW, &_state->original);
  }
  // Exclusive mode belongs to the terminal, not to this descriptor: leaving
  // it set would lock out the next opener for as long as the tty lives.
  if (_config.exclusive) {
    ::ioctl(_handle, TIOCNXCL);
  }
  ::tcflush(_handle, TCIFLUSH);
  ::close(_handle);
}

inline void SerialPort::apply_config(const Config &config) {
  termios options = _state->original;

  // Raw mode as a starting point: no echo, no signals, no input or output
  // post-processing, so that binary data survives untouched. This is the
  // recipe of termios(3), spelled out instead of calling the non-POSIX
  // cfmakeraw(), which needs _DEFAULT_SOURCE to be visible on glibc.
  options.c_iflag &= ~static_cast<tcflag_t>(IGNBRK | BRKINT | PARMRK | ISTRIP |
                                            INLCR | IGNCR | ICRNL | IXON);
  options.c_oflag &= ~static_cast<tcflag_t>(OPOST);
  options.c_lflag &=
      ~static_cast<tcflag_t>(ECHO | ECHOE | ECHONL | ICANON | ISIG | IEXTEN);
  options.c_cflag |= static_cast<tcflag_t>(CREAD | CLOCAL);

  options.c_cflag &= ~static_cast<tcflag_t>(CSIZE);
  switch (config.data_bits) {
  case 5:
    options.c_cflag |= static_cast<tcflag_t>(CS5);
    break;
  case 6:
    options.c_cflag |= static_cast<tcflag_t>(CS6);
    break;
  case 7:
    options.c_cflag |= static_cast<tcflag_t>(CS7);
    break;
  default:
    options.c_cflag |= static_cast<tcflag_t>(CS8);
    break;
  }

  if (config.stop_bits == 2) {
    options.c_cflag |= static_cast<tcflag_t>(CSTOPB);
  } else {
    options.c_cflag &= ~static_cast<tcflag_t>(CSTOPB);
  }

  switch (config.parity) {
  case Parity::even:
    options.c_cflag |= static_cast<tcflag_t>(PARENB);
    options.c_cflag &= ~static_cast<tcflag_t>(PARODD);
    break;
  case Parity::odd:
    options.c_cflag |= static_cast<tcflag_t>(PARENB | PARODD);
    break;
  case Parity::none:
  default:
    options.c_cflag &= ~static_cast<tcflag_t>(PARENB | PARODD);
    break;
  }

  if (config.hangup_on_close) {
    options.c_cflag |= static_cast<tcflag_t>(HUPCL);
  } else {
    options.c_cflag &= ~static_cast<tcflag_t>(HUPCL);
  }

  options.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);
  options.c_iflag &= ~static_cast<tcflag_t>(IXON | IXOFF | IXANY);
  switch (config.flow_control) {
  case FlowControl::software:
    options.c_iflag |= static_cast<tcflag_t>(IXON | IXOFF);
    break;
  case FlowControl::hardware:
    options.c_cflag |= static_cast<tcflag_t>(CRTSCTS);
    break;
  case FlowControl::none:
  default:
    break;
  }

  if (config.canonical_mode) {
    options.c_lflag |= static_cast<tcflag_t>(ICANON);
  }

  // Disable the special characters: data bytes must never be interpreted as
  // editing commands. VSTART and VSTOP are left alone when they are needed by
  // software flow control, and VMIN/VTIME are set last because on some systems
  // they share their storage with VEOF/VEOL.
  for (const int character : {
           VEOF,
           VEOL,
           VEOL2,
           VERASE,
           VWERASE,
           VKILL,
           VREPRINT,
           VINTR,
           VQUIT,
           VSUSP,
           VLNEXT,
           VDISCARD,
#ifdef VSTATUS
           VSTATUS,
#endif
#ifdef VDSUSP
           VDSUSP,
#endif
       }) {
    options.c_cc[character] = _POSIX_VDISABLE;
  }
  if (config.flow_control != FlowControl::software) {
    options.c_cc[VSTART] = _POSIX_VDISABLE;
    options.c_cc[VSTOP] = _POSIX_VDISABLE;
  } else {
    options.c_cc[VSTART] = _state->original.c_cc[VSTART];
    options.c_cc[VSTOP] = _state->original.c_cc[VSTOP];
  }
  // Non-blocking reads: poll(2) is in charge of waiting.
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  const std::optional<speed_t> code = serialport_detail::baud_code(config.baud_rate);
  if (code) {
    if (::cfsetispeed(&options, *code) < 0 ||
        ::cfsetospeed(&options, *code) < 0) {
      serialport_detail::throw_native(
          serialport_detail::last_native_error(),
          "SerialPort: cannot set the baud rate of " + _port);
    }
  }

  if (::tcsetattr(_handle, TCSANOW, &options) < 0) {
    serialport_detail::throw_native(serialport_detail::last_native_error(),
                                    "SerialPort: cannot configure " + _port);
  }

#ifdef SERIALPORT_HAS_CUSTOM_SPEED
  if (!code) {
    speed_t rate = config.baud_rate;
    if (::ioctl(_handle, IOSSIOSPEED, &rate) < 0) {
      serialport_detail::throw_native(
          serialport_detail::last_native_error(),
          "SerialPort: cannot set the baud rate of " + _port + " to " +
              std::to_string(config.baud_rate));
    }
  }
#endif

  // tcsetattr() succeeds when it manages to apply *some* of the settings:
  // read them back to make sure the line is really what was asked for.
  termios applied{};
  if (::tcgetattr(_handle, &applied) < 0) {
    serialport_detail::throw_native(
        serialport_detail::last_native_error(),
        "SerialPort: cannot read back the settings of " + _port);
  }
  constexpr auto mask = static_cast<tcflag_t>(CSIZE | CSTOPB | PARENB | PARODD);
  if ((applied.c_cflag & mask) != (options.c_cflag & mask)) {
    serialport_detail::throw_generic(std::errc::invalid_argument,
                                     "SerialPort: " + _port +
                                         " rejected the requested frame format");
  }
  if (code && ::cfgetospeed(&applied) != *code) {
    serialport_detail::throw_generic(
        std::errc::invalid_argument,
        "SerialPort: " + _port + " rejected the baud rate " +
            std::to_string(config.baud_rate));
  }
}

inline int SerialPort::read_native(char *buf, std::size_t n_max,
                                   const Deadline &deadline) {
  while (true) {
    const int ready = serialport_detail::wait_ready(_handle, false, deadline);
    if (ready < 0) {
      return fail_native(serialport_detail::last_native_error());
    }
    if (ready == 0) {
      return 0;
    }
    const ssize_t n = ::read(_handle, buf, n_max);
    if (n > 0) {
      return serialport_detail::clamp_to_int(static_cast<std::size_t>(n));
    }
    if (n == 0) {
      // poll() reported data but there is none: the device was unplugged.
      return fail_native(ENXIO);
    }
    if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
      return fail_native(serialport_detail::last_native_error());
    }
  }
}

inline int SerialPort::write_native(const char *buf, std::size_t n_bytes,
                                    const Deadline &deadline) {
  while (true) {
    const ssize_t n = ::write(_handle, buf, n_bytes);
    if (n > 0) {
      return serialport_detail::clamp_to_int(static_cast<std::size_t>(n));
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      return fail_native(serialport_detail::last_native_error());
    }
    // The output queue is full (or the driver accepted nothing): wait.
    const int ready = serialport_detail::wait_ready(_handle, true, deadline);
    if (ready < 0) {
      return fail_native(serialport_detail::last_native_error());
    }
    if (ready == 0) {
      return 0;
    }
  }
}

inline std::size_t SerialPort::available_native() const {
  int pending = 0;
  if (::ioctl(_handle, FIONREAD, &pending) < 0) {
    fail_native(serialport_detail::last_native_error());
    return 0;
  }
  return pending > 0 ? static_cast<std::size_t>(pending) : 0;
}

inline void SerialPort::flush_native(bool input, bool output) {
  const int queue = input && output ? TCIOFLUSH : (input ? TCIFLUSH : TCOFLUSH);
  if (::tcflush(_handle, queue) < 0) {
    serialport_detail::throw_native(serialport_detail::last_native_error(),
                                    "SerialPort: cannot flush " + _port);
  }
}

inline void SerialPort::drain_native() {
  while (::tcdrain(_handle) < 0) {
    if (errno != EINTR) {
      serialport_detail::throw_native(serialport_detail::last_native_error(),
                                      "SerialPort: cannot drain " + _port);
    }
  }
}

inline void SerialPort::modem_set_native(ModemLine line, bool on) {
  int bits = 0;
  switch (line) {
  case ModemLine::dtr:
    bits = TIOCM_DTR;
    break;
  case ModemLine::rts:
    bits = TIOCM_RTS;
    break;
  default:
    serialport_detail::throw_generic(std::errc::invalid_argument,
                                     "SerialPort: not an output line on " +
                                         _port);
  }
  if (::ioctl(_handle, on ? TIOCMBIS : TIOCMBIC, &bits) < 0) {
    serialport_detail::throw_native(
        serialport_detail::last_native_error(),
        "SerialPort: cannot set the modem lines of " + _port);
  }
}

inline bool SerialPort::modem_get_native(ModemLine line) const {
  int bits = 0;
  if (::ioctl(_handle, TIOCMGET, &bits) < 0) {
    serialport_detail::throw_native(
        serialport_detail::last_native_error(),
        "SerialPort: cannot read the modem lines of " + _port);
  }
  switch (line) {
  case ModemLine::cts:
    return (bits & TIOCM_CTS) != 0;
  case ModemLine::dsr:
    return (bits & TIOCM_DSR) != 0;
  case ModemLine::dcd:
    return (bits & TIOCM_CAR) != 0;
  default:
    serialport_detail::throw_generic(std::errc::invalid_argument,
                                     "SerialPort: not an input line on " +
                                         _port);
  }
}

inline void SerialPort::break_native(int duration) {
  if (::tcsendbreak(_handle, duration) < 0) {
    serialport_detail::throw_native(serialport_detail::last_native_error(),
                                    "SerialPort: cannot send a break on " +
                                        _port);
  }
}

inline std::vector<std::string> SerialPort::available_ports() {
  namespace fs = std::filesystem;
  std::vector<std::string> ports;
  std::error_code ec;
  const fs::directory_iterator end;

#ifdef __linux__
  // Only the ttys backed by a driver are real ports.
  for (fs::directory_iterator it("/sys/class/tty", ec); !ec && it != end;
       it.increment(ec)) {
    std::error_code link_ec;
    if (fs::exists(it->path() / "device" / "driver", link_ec)) {
      ports.push_back("/dev/" + it->path().filename().string());
    }
  }
  if (!ports.empty()) {
    std::ranges::sort(ports);
    return ports;
  }
  ec.clear();
#endif

  for (fs::directory_iterator it("/dev", ec); !ec && it != end;
       it.increment(ec)) {
    const std::string name = it->path().filename().string();
    const std::string_view entry{name};
#ifdef __APPLE__
    // The callout devices (cu.*) do not wait for the carrier, unlike tty.*.
    const bool candidate = entry.starts_with("cu.");
#else
    const bool candidate =
        entry.starts_with("ttyUSB") || entry.starts_with("ttyACM") ||
        entry.starts_with("ttyAMA") || entry.starts_with("ttyS") ||
        entry.starts_with("rfcomm");
#endif
    if (candidate) {
      ports.push_back(it->path().string());
    }
  }
  std::ranges::sort(ports);
  return ports;
}

#endif /* _WIN32 */

/* WRITING ******************************************************************/

inline int SerialPort::write(const char *buf, std::size_t n_bytes) {
  begin_operation();
  if (!is_open()) {
    return fail(std::errc::bad_file_descriptor);
  }
  if (n_bytes == 0) {
    return 0;
  }
  if (buf == nullptr) {
    return fail(std::errc::bad_address);
  }

  const Deadline deadline = serialport_detail::make_deadline(_config.timeout);
  std::size_t written = 0;
  while (written < n_bytes) {
    const int n = write_native(buf + written, n_bytes - written, deadline);
    if (n < 0) {
      return written > 0 ? serialport_detail::clamp_to_int(written) : -1;
    }
    if (n == 0) {
      _timed_out = true;
      break;
    }
    written += static_cast<std::size_t>(n);
  }
  return serialport_detail::clamp_to_int(written);
}

inline int SerialPort::write(std::string_view data) {
  return write(data.data(), data.size());
}

inline int SerialPort::write(std::span<const std::byte> data) {
  return write(reinterpret_cast<const char *>(data.data()), data.size());
}

inline int SerialPort::write_line(std::string_view data, std::string_view eol) {
  // A single write keeps the line from being interleaved with other output.
  std::string buffer;
  buffer.reserve(data.size() + eol.size());
  buffer.append(data).append(eol);
  return write(buffer);
}

/* READING ******************************************************************/

inline int SerialPort::read(char *buf, std::size_t n_bytes) {
  begin_operation();
  _pending_lf = false;
  if (!is_open()) {
    return fail(std::errc::bad_file_descriptor);
  }
  if (n_bytes == 0) {
    return 0;
  }
  if (buf == nullptr) {
    return fail(std::errc::bad_address);
  }

  const Deadline deadline = serialport_detail::make_deadline(_config.timeout);
  std::size_t count = 0;
  while (count < n_bytes) {
    count += take_buffered(buf + count, n_bytes - count);
    if (count == n_bytes) {
      break;
    }
    const int ret = fill_buffer(deadline);
    if (ret < 0) {
      return count > 0 ? serialport_detail::clamp_to_int(count) : -1;
    }
    if (ret == 0) {
      _timed_out = true;
      break;
    }
  }
  return serialport_detail::clamp_to_int(count);
}

inline int SerialPort::read(std::span<char> buf) {
  return read(buf.data(), buf.size());
}

inline int SerialPort::read(std::span<std::byte> buf) {
  return read(reinterpret_cast<char *>(buf.data()), buf.size());
}

inline int SerialPort::read_some(char *buf, std::size_t n_max) {
  begin_operation();
  _pending_lf = false;
  if (!is_open()) {
    return fail(std::errc::bad_file_descriptor);
  }
  if (n_max == 0) {
    return 0;
  }
  if (buf == nullptr) {
    return fail(std::errc::bad_address);
  }

  if (buffered() == 0) {
    const int ret = fill_buffer(serialport_detail::make_deadline(_config.timeout));
    if (ret < 0) {
      return -1;
    }
    if (ret == 0) {
      _timed_out = true;
      return 0;
    }
  }
  return serialport_detail::clamp_to_int(take_buffered(buf, n_max));
}

inline int SerialPort::read_line(std::string &line) {
  begin_operation();
  line.clear();
  if (!is_open()) {
    return fail(std::errc::bad_file_descriptor);
  }

  const Deadline deadline = serialport_detail::make_deadline(_config.timeout);
  while (true) {
    while (_rx_head < _rx_tail) {
      const char c = _rx_buf[_rx_head++];
      if (_pending_lf) {
        // The previous line ended with CR: swallow the LF of a CRLF pair.
        _pending_lf = false;
        if (c == '\n') {
          continue;
        }
      }
      if (c == '\n') {
        return serialport_detail::clamp_to_int(line.size());
      }
      if (c == '\r') {
        _pending_lf = true;
        return serialport_detail::clamp_to_int(line.size());
      }
      if (line.size() >= _config.max_line_length) {
        // Runaway line: keep draining until the terminator, but stop growing.
        fail(std::errc::message_size);
        continue;
      }
      line.push_back(c);
    }
    const int ret = fill_buffer(deadline);
    if (ret < 0) {
      return line.empty() ? -1 : serialport_detail::clamp_to_int(line.size());
    }
    if (ret == 0) {
      _timed_out = true;
      return serialport_detail::clamp_to_int(line.size());
    }
  }
}

inline int SerialPort::read_line(char *line, std::size_t n_max) {
  if (line == nullptr || n_max == 0) {
    begin_operation();
    return fail(std::errc::invalid_argument);
  }

  const int ret = read_line(_line_buf);
  if (ret < 0) {
    line[0] = '\0';
    return ret;
  }
  const std::size_t count = std::min(_line_buf.size(), n_max - 1);
  if (count < _line_buf.size()) {
    fail(std::errc::message_size);
  }
  std::memcpy(line, _line_buf.data(), count);
  line[count] = '\0';
  return serialport_detail::clamp_to_int(count);
}

inline std::optional<std::string> SerialPort::read_line() {
  std::string line;
  if (read_line(line) < 0 || _timed_out) {
    return std::nullopt;
  }
  return line;
}

/* PORT STATE ***************************************************************/

inline void SerialPort::set_baud_rate(unsigned value) {
  if (!is_open()) {
    serialport_detail::throw_generic(std::errc::bad_file_descriptor,
                                     "SerialPort: port is closed");
  }
  Config updated = _config;
  updated.baud_rate = value;
  serialport_detail::validate(updated, _port);
  drain();
  try {
    apply_config(updated);
  } catch (...) {
    // Leave the port on the rate it was working at.
    try {
      apply_config(_config);
    } catch (...) {
    }
    throw;
  }
  _config = updated;
}

inline std::size_t SerialPort::available() const {
  std::size_t count = buffered();
  if (is_open()) {
    count += available_native();
  }
  return count;
}

inline void SerialPort::flush_input() {
  _rx_head = 0;
  _rx_tail = 0;
  _pending_lf = false;
  if (!is_open()) {
    serialport_detail::throw_generic(std::errc::bad_file_descriptor,
                                     "SerialPort: port is closed");
  }
  flush_native(true, false);
}

inline void SerialPort::flush_output() {
  if (!is_open()) {
    serialport_detail::throw_generic(std::errc::bad_file_descriptor,
                                     "SerialPort: port is closed");
  }
  flush_native(false, true);
}

inline void SerialPort::flush() {
  _rx_head = 0;
  _rx_tail = 0;
  _pending_lf = false;
  if (!is_open()) {
    serialport_detail::throw_generic(std::errc::bad_file_descriptor,
                                     "SerialPort: port is closed");
  }
  flush_native(true, true);
}

inline void SerialPort::drain() {
  if (!is_open()) {
    serialport_detail::throw_generic(std::errc::bad_file_descriptor,
                                     "SerialPort: port is closed");
  }
  drain_native();
}

/* MODEM LINES **************************************************************/

inline void SerialPort::set_dtr(bool on) {
  if (!is_open()) {
    serialport_detail::throw_generic(std::errc::bad_file_descriptor,
                                     "SerialPort: port is closed");
  }
  modem_set_native(ModemLine::dtr, on);
}

inline void SerialPort::set_rts(bool on) {
  if (!is_open()) {
    serialport_detail::throw_generic(std::errc::bad_file_descriptor,
                                     "SerialPort: port is closed");
  }
  modem_set_native(ModemLine::rts, on);
}

inline bool SerialPort::cts() const {
  if (!is_open()) {
    serialport_detail::throw_generic(std::errc::bad_file_descriptor,
                                     "SerialPort: port is closed");
  }
  return modem_get_native(ModemLine::cts);
}

inline bool SerialPort::dsr() const {
  if (!is_open()) {
    serialport_detail::throw_generic(std::errc::bad_file_descriptor,
                                     "SerialPort: port is closed");
  }
  return modem_get_native(ModemLine::dsr);
}

inline bool SerialPort::dcd() const {
  if (!is_open()) {
    serialport_detail::throw_generic(std::errc::bad_file_descriptor,
                                     "SerialPort: port is closed");
  }
  return modem_get_native(ModemLine::dcd);
}

inline void SerialPort::send_break(int duration) {
  if (!is_open()) {
    serialport_detail::throw_generic(std::errc::bad_file_descriptor,
                                     "SerialPort: port is closed");
  }
  break_native(duration);
}

/* INTERNALS ****************************************************************/

inline int SerialPort::fill_buffer(const Deadline &deadline) {
  if (_rx_head == _rx_tail) {
    _rx_head = 0;
    _rx_tail = 0;
  } else if (_rx_tail == _rx_buf.size()) {
    const std::size_t left = buffered();
    std::memmove(_rx_buf.data(), _rx_buf.data() + _rx_head, left);
    _rx_head = 0;
    _rx_tail = left;
  }
  if (_rx_tail == _rx_buf.size()) {
    return fail(std::errc::no_buffer_space);
  }

  const int n = read_native(_rx_buf.data() + _rx_tail,
                            _rx_buf.size() - _rx_tail, deadline);
  if (n > 0) {
    _rx_tail += static_cast<std::size_t>(n);
  }
  return n;
}

inline std::size_t SerialPort::take_buffered(char *dst,
                                             std::size_t n_max) noexcept {
  const std::size_t count = std::min(buffered(), n_max);
  if (count > 0) {
    std::memcpy(dst, _rx_buf.data() + _rx_head, count);
    _rx_head += count;
    if (_rx_head == _rx_tail) {
      _rx_head = 0;
      _rx_tail = 0;
    }
  }
  return count;
}

inline int SerialPort::fail(std::errc code) const noexcept {
  _last_error = std::make_error_code(code);
  return -1;
}

inline int SerialPort::fail_native(int error_number) const noexcept {
  _last_error = std::error_code(error_number, std::system_category());
  return -1;
}

inline void SerialPort::begin_operation() noexcept {
  _timed_out = false;
  _last_error.clear();
}

#ifdef SERIALPORT_HAS_CUSTOM_SPEED
#undef SERIALPORT_HAS_CUSTOM_SPEED
#endif

#endif /* SERIALPORT_HPP */
