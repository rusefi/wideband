#pragma once


#include <cstdint>

#include "port.h"


using SendMessageCallback = void (*)(Configuration*, uint8_t);
constexpr uint8_t kUnusedChannel = 0xFF;

struct ProtocolHandler final {
    uint16_t interval_ms;
    SendMessageCallback send_message;
};

/*
Reference usage:

// Protocol source file example (e.g. can_haltech.cpp):
// -----------------------------------------------------
// static void SendHaltechAfr(Configuration* cfg)
// {
//     // ...
// }
//
// // Actual descriptor stays file-local constexpr in flash.
// constexpr ProtocolHandler kHaltechAfrTxHandler = MakeProtocolHandler<&SendHaltechAfr>(50);
//
// Protocol header (e.g. can_haltech.h):
// -------------------------------------
// extern const ProtocolHandler& haltechAfrTxHandler;
*/

template<void (*Send)(Configuration*)>
inline void SendMessageAdapter(Configuration* configuration, uint8_t)
{
    Send(configuration);
}

template<void (*Send)(Configuration*, uint8_t)>
inline void SendMessageChannelAdapter(Configuration* configuration, uint8_t channel)
{
    Send(configuration, channel);
}

template<void (*Send)(Configuration*)>
constexpr ProtocolHandler MakeProtocolHandler(uint16_t intervalMs)
{
    return { intervalMs, &SendMessageAdapter<Send> };
}

template<void (*Send)(Configuration*, uint8_t)>
constexpr ProtocolHandler MakeProtocolHandler(uint16_t intervalMs)
{
    return { intervalMs, &SendMessageChannelAdapter<Send> };
}

inline void DispatchProtocolHandler(
    const ProtocolHandler& handler,
    uint16_t elapsedMs,
    uint16_t& elapsedSinceDispatchMs,
    Configuration* configuration,
    uint8_t channel)
{
    elapsedSinceDispatchMs += elapsedMs;
    if (elapsedSinceDispatchMs < handler.interval_ms) {
        return;
    }

    elapsedSinceDispatchMs -= handler.interval_ms;
    handler.send_message(configuration, channel);
}

inline void DispatchProtocolHandler(
    const ProtocolHandler& handler,
    uint16_t elapsedMs,
    uint16_t& elapsedSinceDispatchMs,
    Configuration* configuration)
{
    DispatchProtocolHandler(handler, elapsedMs, elapsedSinceDispatchMs, configuration, kUnusedChannel);
}
