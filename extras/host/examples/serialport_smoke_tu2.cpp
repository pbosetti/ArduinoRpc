/* Second translation unit that also includes serialport.hpp and calls a
 * SerialPort member, so that linking this file together with
 * serialport_smoke.cpp proves the header has no ODR / multiple-definition
 * problems when included from more than one .cpp file. */

#include "serialport.hpp"

#include <cstddef>

/** Trivial function, defined in a second TU, that exercises a SerialPort
 * static member. Only its return value is used by main(); its purpose is to
 * force the compiler to instantiate SerialPort::available_ports() (and the
 * rest of the header) a second time in a distinct object file. */
std::size_t tu2_port_count() { return SerialPort::available_ports().size(); }
