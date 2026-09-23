/* Smoke test for the header-only host/include/serialport.hpp.
 *
 * This program, together with serialport_smoke_tu2.cpp, includes the header
 * from two separate translation units and links them into one executable:
 * if the header were missing `inline` anywhere, the link step would fail
 * with a multiple-definition error. Beyond that link-time check, running
 * the program lists the serial ports found on the machine and, if a port
 * path is given on the command line, opens it at 115200 baud with a 500 ms
 * timeout and prints up to 5 lines read from it (a timeout is reported but
 * is not treated as a failure).
 */

#include "serialport.hpp"

#include <cstddef>
#include <cstdio>
#include <exception>
#include <optional>
#include <string>

// Defined in serialport_smoke_tu2.cpp: proves the header also works, and
// still links cleanly, when instantiated a second time in another TU.
std::size_t tu2_port_count();

int main(int argc, char *argv[]) {
  const auto ports = SerialPort::available_ports();
  std::printf("available_ports(): %zu port(s) (tu2 sees %zu)\n", ports.size(),
              tu2_port_count());
  for (const auto &port : ports) {
    std::printf("  %s\n", port.c_str());
  }

  if (argc < 2) {
    std::printf("no port given on the command line, skipping the open test\n");
    return 0;
  }

  try {
    SerialPort port(argv[1], {.baud_rate = 115200,
                              .timeout = SerialPort::Milliseconds{500}});
    std::printf("opened %s at %u baud\n", port.port().c_str(), port.baud_rate());

    for (int i = 0; i < 5; ++i) {
      const std::optional<std::string> line = port.read_line();
      if (!line) {
        if (port.timed_out()) {
          std::printf("read_line(): timed out, stopping\n");
        } else {
          std::printf("read_line(): error: %s\n",
                      port.last_error().message().c_str());
        }
        break;
      }
      std::printf("line %d: %s\n", i, line->c_str());
    }
  } catch (const std::exception &e) {
    std::fprintf(stderr, "could not open %s: %s\n", argv[1], e.what());
    return 1;
  }

  return 0;
}
