/// \file SerialRPC.h
/// \brief Umbrella include for the SerialRPC Arduino library.
///
/// At this stage of the build only the shared MsgPack-lite codec exists.
/// `serial_rpc/framing.h` (COBS + CRC16 + Demux) and `serial_rpc/server.h`
/// (the device-side dispatcher) are added in later steps of the
/// implementation plan; see docs/PLAN.md at the repository root.
#pragma once

#include "serial_rpc/msgpack_lite.h"
