/// \file SerialRPC.h
/// \brief Umbrella include for the SerialRPC Arduino library.
///
/// Includes the shared MsgPack-lite codec, the COBS + CRC16 + Demux framing
/// layer, and the device-side dispatcher (`serial_rpc::Server`, plus the
/// `SerialRPC<>` convenience alias on Arduino). See docs/PLAN.md at the
/// repository root for the design.
#pragma once

#include "serial_rpc/config.h"
#include "serial_rpc/msgpack_lite.h"
#include "serial_rpc/framing.h"
#include "serial_rpc/server.h"
