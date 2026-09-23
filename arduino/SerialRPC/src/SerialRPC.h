/// \file SerialRPC.h
/// \brief Umbrella include for the SerialRPC Arduino library.
///
/// At this stage of the build the shared MsgPack-lite codec and the COBS
/// + CRC16 + Demux framing layer exist. `serial_rpc/server.h` (the
/// device-side dispatcher) is added in a later step of the implementation
/// plan; see docs/PLAN.md at the repository root.
#pragma once

#include "serial_rpc/msgpack_lite.h"
#include "serial_rpc/framing.h"
