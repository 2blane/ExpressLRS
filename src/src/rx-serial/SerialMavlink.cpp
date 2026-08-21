#if defined(TARGET_RX)

#include "SerialMavlink.h"
#include "OTA.h"
#include "common.h"
#include "config.h"
#include "device.h"

#define MAVLINK_RC_PACKET_INTERVAL 10

#define MAVLINK_COMM_NUM_BUFFERS 2
#include "common/mavlink.h"

#define MAV_FTP_OPCODE_OPENFILERO 4

#if defined(STARBOUND_RECEIVER)
extern void starboundReceiverSetBroadcastMode(bool enabled);
extern bool starboundReceiverIsBroadcastMode();
extern uint32_t starboundReceiverLastValidPacket();
extern uint16_t starboundReceiverHardwareErrors();
extern uint16_t starboundReceiverCrcErrors();

static bool processStarboundModeCommand(const mavlink_message_t &msg)
{
    constexpr float ELRS_MODE_CHANGE = 8.0f;
    constexpr float ELRS_NORMAL_MODE = 0.0f;
    constexpr float ELRS_BROADCAST_RECEIVE_MODE = 2.0f;

    uint16_t commandId;
    float marker;
    float requestedMode;
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT)
    {
        mavlink_command_int_t command{};
        mavlink_msg_command_int_decode(&msg, &command);
        commandId = command.command;
        marker = command.param1;
        requestedMode = command.param2;
    }
    else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG)
    {
        mavlink_command_long_t command{};
        mavlink_msg_command_long_decode(&msg, &command);
        commandId = command.command;
        marker = command.param1;
        requestedMode = command.param2;
    }
    else
    {
        return false;
    }

    if (commandId != MAV_CMD_USER_1 || marker != ELRS_MODE_CHANGE)
    {
        return false;
    }

    if (requestedMode == ELRS_NORMAL_MODE)
    {
        starboundReceiverSetBroadcastMode(false);
        return true;
    }
    if (requestedMode == ELRS_BROADCAST_RECEIVE_MODE)
    {
        starboundReceiverSetBroadcastMode(true);
        return true;
    }
    return false;
}
#endif

SerialMavlink::SerialMavlink(Stream &out, Stream &in):
    SerialIO(&out, &in),

    //system ID of the device component sending command to FC, can be set using lua options, 0 is the default value for initialized storage, treat it as 255 which is commonly used as GCS SysID
    this_system_id(config.GetSourceSysId() ? config.GetSourceSysId() : 255),
    //use telemetry radio compId as we are providing radio status messages and pass telemetry
    this_component_id(MAV_COMPONENT::MAV_COMP_ID_TELEMETRY_RADIO),

    // system ID of vehicle we want to control must be the same as target vehicle, can be set using lua options, 0 is the default value for initialized storage, treat it as 1 which is commonly used as UAV SysID in 1:1 networks
    target_system_id(config.GetTargetSysId() ? config.GetTargetSysId() : 1),
    // Send to all components as we may have ex. gimbal that listens to RC instead of using Autopilot driver
    target_component_id(MAV_COMPONENT::MAV_COMP_ID_ALL)
{
}

uint32_t SerialMavlink::sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData)
{
    if (!frameAvailable) {
        return DURATION_IMMEDIATELY;
    }

    const mavlink_rc_channels_override_t rc_override {
        chan1_raw: CRSF_to_US(channelData[0]),
        chan2_raw: CRSF_to_US(channelData[1]),
        chan3_raw: CRSF_to_US(channelData[2]),
        chan4_raw: CRSF_to_US(channelData[3]),
        chan5_raw: CRSF_to_US(channelData[4]),
        chan6_raw: CRSF_to_US(channelData[5]),
        chan7_raw: CRSF_to_US(channelData[6]),
        chan8_raw: CRSF_to_US(channelData[7]),
        target_system: target_system_id,
        target_component: target_component_id,
        chan9_raw: CRSF_to_US(channelData[8]),
        chan10_raw: CRSF_to_US(channelData[9]),
        chan11_raw: CRSF_to_US(channelData[10]),
        chan12_raw: CRSF_to_US(channelData[11]),
        chan13_raw: CRSF_to_US(channelData[12]),
        chan14_raw: CRSF_to_US(channelData[13]),
        chan15_raw: CRSF_to_US(channelData[14]),
        chan16_raw: CRSF_to_US(channelData[15]),
    };

    uint8_t buf[MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES];
    mavlink_message_t msg;
    mavlink_msg_rc_channels_override_encode(this_system_id, this_component_id, &msg, &rc_override);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    _outputPort->write(buf, len);

    return MAVLINK_RC_PACKET_INTERVAL;
}

int SerialMavlink::getMaxSerialReadSize()
{
    return MAV_INPUT_BUF_LEN - mavlinkInputBuffer.size();
}

void SerialMavlink::processBytes(uint8_t *bytes, u_int16_t size)
{
#if defined(STARBOUND_RECEIVER)
    bool handledModeCommand = false;
    for (uint16_t i = 0; i < size; ++i)
    {
        mavlink_message_t msg;
        mavlink_status_t status;
        if (mavlink_frame_char(MAVLINK_COMM_1, bytes[i], &msg, &status) == MAVLINK_FRAMING_OK)
        {
            handledModeCommand |= processStarboundModeCommand(msg);
        }
    }

    // Local flight-controller traffic must never enter the RF downlink while
    // receive-only mode is active. Also consume the mode command locally.
    if (starboundReceiverIsBroadcastMode() || handledModeCommand)
    {
        return;
    }
#endif
    if (connectionState == connected)
    {
        mavlinkInputBuffer.atomicPushBytes(bytes, size);
    }
}

void SerialMavlink::sendQueuedData(uint32_t maxBytesToSend)
{
    const uint32_t now = millis();
#if defined(STARBOUND_RECEIVER)
    const bool broadcastMode = starboundReceiverIsBroadcastMode();
    constexpr uint32_t BROADCAST_RADIO_STATUS_INTERVAL = 1000;
    constexpr uint32_t BROADCAST_SIGNAL_ACTIVE_TIMEOUT = 2000;
    const uint32_t lastValidBroadcastPacket = starboundReceiverLastValidPacket();
    const bool broadcastSignalActive = lastValidBroadcastPacket != 0 &&
        (now - lastValidBroadcastPacket) <= BROADCAST_SIGNAL_ACTIVE_TIMEOUT;
    const uint32_t radioStatusInterval = broadcastMode ? BROADCAST_RADIO_STATUS_INTERVAL : 10;
    const uint16_t broadcastHardwareErrors = broadcastMode ? starboundReceiverHardwareErrors() : 0;
    const uint16_t broadcastCrcErrors = broadcastMode ? starboundReceiverCrcErrors() : 0;
#else
    constexpr bool broadcastMode = false;
    constexpr bool broadcastSignalActive = false;
    constexpr uint32_t radioStatusInterval = 10;
    constexpr uint16_t broadcastHardwareErrors = 0;
    constexpr uint16_t broadcastCrcErrors = 0;
#endif

    // Normal MAVLink mode retains its 100 Hz flow-control report. Broadcast
    // mode sends one status per second so the STM32 can verify the UART even
    // when the Ranger is absent; inactive RF measurements are reported as 0.
    if ((now - lastSentFlowCtrl) > radioStatusInterval)
    {
        lastSentFlowCtrl = now;

        // Software-based flow control for mavlink
        uint8_t percentage_remaining = ((MAV_INPUT_BUF_LEN - mavlinkInputBuffer.size()) * 100) / MAV_INPUT_BUF_LEN;
        const bool broadcastSignalInactive = broadcastMode && !broadcastSignalActive;
        const uint8_t statusRssi = broadcastSignalInactive ? uint8_t{0} :
            (uint8_t)((float)linkStats.uplink_Link_quality * 2.55);
        const uint8_t statusRemoteRssi = broadcastSignalInactive ? uint8_t{0} :
            (uint8_t)linkStats.uplink_RSSI_1;
        const uint8_t statusNoise = broadcastSignalInactive ? uint8_t{0} :
            (uint8_t)linkStats.uplink_SNR;

        // Populate radio status packet
        const mavlink_radio_status_t radio_status {
            rxerrors: broadcastHardwareErrors,
            fixed: broadcastCrcErrors,
            rssi: statusRssi,
            remrssi: statusRemoteRssi,
            txbuf: percentage_remaining,
            noise: statusNoise,
            remnoise: 0,
        };

        uint8_t buf[MAVLINK_MSG_ID_RADIO_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES];
        mavlink_message_t msg;
        mavlink_msg_radio_status_encode(this_system_id, this_component_id, &msg, &radio_status);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        _outputPort->write(buf, len);
    }

    auto size = mavlinkOutputBuffer.size();
    if (size == 0)
    {
        // nothing to send
        return;
    }

    uint8_t apBuf[size];
    mavlinkOutputBuffer.lock();
    mavlinkOutputBuffer.popBytes(apBuf, size);
    mavlinkOutputBuffer.unlock();

    for (uint8_t i = 0; i < size; ++i)
    {
        uint8_t c = apBuf[i];

        mavlink_message_t msg;
        mavlink_status_t status;

        // Try parse a mavlink message
        if (mavlink_frame_char(MAVLINK_COMM_0, c, &msg, &status))
        {
            // Message decoded successfully

            // Forward message to the UART
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
            _outputPort->write(buf, len);
        }
    }
}

void SerialMavlink::event()
{
    this_system_id = config.GetSourceSysId() ? config.GetSourceSysId() : 255;
    target_system_id = config.GetTargetSysId() ? config.GetTargetSysId() : 1;
}

void SerialMavlink::forwardMessage(const uint8_t *data)
{
#if defined(STARBOUND_RECEIVER)
    if (starboundReceiverIsBroadcastMode())
    {
        lastBroadcastMessageReceived = millis();
    }
#endif
    mavlinkOutputBuffer.atomicPushBytes(data + 2, data[1]);
}

bool SerialMavlink::GetNextPayload(uint8_t* nextPayloadSize, uint8_t *payloadData)
{
    if (mavlinkInputBuffer.size() == 0)
    {
        return false;
    }
    const uint16_t count = std::min(mavlinkInputBuffer.size(), (uint16_t)CRSF_PAYLOAD_SIZE_MAX); // Constrain to CRSF max payload size to match SS
    payloadData[0] = CRSF_ADDRESS_USB; // device_addr - used on TX to differentiate between std tlm and mavlink
    payloadData[1] = count;
    // The following 'n' bytes are just raw mavlink
    mavlinkInputBuffer.popBytes(payloadData + CRSF_FRAME_NOT_COUNTED_BYTES, count);
    *nextPayloadSize = count + CRSF_FRAME_NOT_COUNTED_BYTES;
    return true;
}

void SerialMavlink::ResetState()
{
    mavlinkInputBuffer.flush();
    mavlinkOutputBuffer.flush();
#if defined(STARBOUND_RECEIVER)
    lastBroadcastMessageReceived = 0;
#endif
}

#endif // defined(TARGET_RX)
