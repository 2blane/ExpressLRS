#pragma once

#include "SerialIO.h"
#include "FIFO.h"

#define MAV_INPUT_BUF_LEN       1024
#define MAV_OUTPUT_BUF_LEN      512
#define MAV_SERIAL_OUTPUT_BUF_LEN 2048
#define MAV_PAYLOAD_SIZE_MAX    60

// Variables / constants
extern FIFO<MAV_INPUT_BUF_LEN> mavlinkInputBuffer;
extern FIFO<MAV_OUTPUT_BUF_LEN> mavlinkOutputBuffer;

class SerialMavlink final : public SerialIO {
public:
    explicit SerialMavlink(Stream &out, Stream &in);
    ~SerialMavlink() override = default;

    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;

    int getMaxSerialReadSize() override;
    void sendQueuedData(uint32_t maxBytesToSend) override;

    void forwardMessage(const uint8_t *data);
    bool GetNextPayload(uint8_t *nextPayloadSize, uint8_t *payloadData);
    void ResetState();

    void event() override;

private:
    void processBytes(uint8_t *bytes, u_int16_t size) override;
    bool queueSerialBytes(const uint8_t *data, uint16_t len);
    void drainSerialOutput(uint32_t maxBytesToSend);

    uint8_t this_system_id;
    const uint8_t this_component_id;

    uint8_t target_system_id;
    const uint8_t target_component_id;

    uint32_t lastSentFlowCtrl = 0;
#if defined(STARBOUND_RECEIVER)
    uint32_t lastBroadcastMessageReceived = 0;
    uint32_t lastBroadcastDiagnostic = 0;
    uint8_t broadcastDiagnosticPage = 0;
#endif

    // Variables / constants for Mavlink //
    FIFO<MAV_INPUT_BUF_LEN> mavlinkInputBuffer;
    FIFO<MAV_OUTPUT_BUF_LEN> mavlinkOutputBuffer;
    FIFO<MAV_SERIAL_OUTPUT_BUF_LEN> mavlinkSerialOutputBuffer;
};
