#pragma once


#include <cstdint>

#include "port.h"

class BaseProtocolHandler {
public:
    explicit BaseProtocolHandler(uint16_t intervalMs)
        : m_intervalMs(intervalMs)
    {
    }

    virtual uint16_t interval_ms() const
    {
        return m_intervalMs;
    }

    virtual bool is_enabled(const Configuration*) const
    {
        return true;
    }

    virtual void send_message(Configuration*)
    {
    }

    virtual void dispatch(uint32_t elapsedMs, Configuration* configuration)
    {
        if (!is_enabled(configuration)) {
            return;
        }

        auto interval = interval_ms();
        m_elapsedSinceDispatchMs += elapsedMs;
        if (m_elapsedSinceDispatchMs < interval) {
            return;
        }

        m_elapsedSinceDispatchMs -= interval;

        send_message(configuration);
    }

    uint16_t m_intervalMs;

private:
    uint32_t m_elapsedSinceDispatchMs = 0;
};

class AfrHandler : public BaseProtocolHandler {
public:
    using SendAfrCallback = void (*)(Configuration*, uint8_t);

    AfrHandler(CanAfrProtocol protocol, uint16_t intervalMs, SendAfrCallback send)
        : BaseProtocolHandler(intervalMs)
        , m_protocol(protocol)
        , m_send(send)
    {
    }

    void send_message(Configuration* configuration) override
    {
        for (uint8_t ch = 0; ch < AFR_CHANNELS; ch++) {
            if (configuration->afr[ch].ExtraCanProtocol != m_protocol) {
                continue;
            }

            m_send(configuration, ch);
        }
    }

private:
    const CanAfrProtocol m_protocol;
    SendAfrCallback m_send;
};

class EgtHandler : public BaseProtocolHandler {
public:
    using SendEgtCallback = void (*)(Configuration*);

    EgtHandler(CanEgtProtocol protocol, uint16_t intervalMs, SendEgtCallback send)
        : BaseProtocolHandler(intervalMs)
        , m_protocol(protocol)
        , m_send(send)
    {
    }

    bool is_enabled(const Configuration* configuration) const override
    {
#if (EGT_CHANNELS > 0)
        // Base egt handler just check first channel is enabled for this protocol
        return configuration->egt[0].ExtraCanProtocol == m_protocol;
#else
        (void)configuration;
#endif

        return false;
    }

    void send_message(Configuration* configuration) override
    {
        m_send(configuration);
    }

private:
    const CanEgtProtocol m_protocol;
    SendEgtCallback m_send;
};

class IoHandler final : public BaseProtocolHandler {
public:
    using SendIoCallback = void (*)(Configuration*);

    IoHandler(CanIoProtocol protocol, uint16_t intervalMs, SendIoCallback send)
        : BaseProtocolHandler(intervalMs)
        , m_protocol(protocol)
        , m_send(send)
    {
    }

    bool is_enabled(const Configuration* configuration) const override
    {
        #if (IO_EXPANDER_ENABLED > 0)
        return configuration->ioExpanderConfig.Protocol == m_protocol;
        #else
        (void)configuration;
        return false;
        #endif
    }

    void send_message(Configuration* configuration) override
    {
        m_send(configuration);
    }

private:
    const CanIoProtocol m_protocol;
    SendIoCallback m_send;
};

class CallbackHandler final : public BaseProtocolHandler {
public:
    using IsEnabledCallback = bool (*)(const Configuration*);
    using SendCallback = void (*)(Configuration*);

    CallbackHandler(uint16_t intervalMs, IsEnabledCallback isEnabled, SendCallback send)
        : BaseProtocolHandler(intervalMs)
        , m_isEnabled(isEnabled)
        , m_send(send)
    {
    }

    bool is_enabled(const Configuration* configuration) const override
    {
        return m_isEnabled(configuration);
    }

    void send_message(Configuration* configuration) override
    {
        m_send(configuration);
    }

private:
    IsEnabledCallback m_isEnabled;
    SendCallback m_send;
};
