#if defined(TARGET_RX)

#define MAVLINK_COMM_NUM_BUFFERS 2

#include "SerialMavlink.h"
#include "OTA.h"
#include "common.h"
#include "config.h"
#include "device.h"
#include "MAVLink.h"

#define MAVLINK_RC_PACKET_INTERVAL 10

#include "common/mavlink.h"

#define MAV_FTP_OPCODE_OPENFILERO 4

#if defined(STARBOUND_RECEIVER)
extern void starboundReceiverSetBroadcastMode(bool enabled);
extern bool starboundReceiverIsBroadcastMode();
extern bool starboundReceiverPilotUsesMavlinkOta();
extern uint32_t starboundReceiverLastValidPacket();
extern uint16_t starboundReceiverHardwareErrors();
extern uint16_t starboundReceiverCrcErrors();

static constexpr uint8_t STARBOUND_RADIO_MODE_PILOT = 0xB0;
static constexpr uint8_t STARBOUND_RADIO_MODE_BROADCAST = 0xB2;

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

#if defined(STARBOUND_RECEIVER)
    // ArduPilot accepts RC_CHANNELS_OVERRIDE only from SYSID_MYGCS. Starbound
    // fixes that to 255 and broadcasts only on the receiver's dedicated local
    // UART, so persisted ELRS IDs cannot silently block Pilot controls and the
    // same receiver firmware works with every drone SYSID.
    this_system_id(255),
#else
    //system ID of the device component sending command to FC, can be set using lua options, 0 is the default value for initialized storage, treat it as 255 which is commonly used as GCS SysID
    this_system_id(config.GetSourceSysId() ? config.GetSourceSysId() : 255),
#endif
    //use telemetry radio compId as we are providing radio status messages and pass telemetry
    this_component_id(MAV_COMPONENT::MAV_COMP_ID_TELEMETRY_RADIO),

    // system ID of vehicle we want to control must be the same as target vehicle, can be set using lua options, 0 is the default value for initialized storage, treat it as 1 which is commonly used as UAV SysID in 1:1 networks
#if defined(STARBOUND_RECEIVER)
    target_system_id(1),
#else
    target_system_id(config.GetTargetSysId() ? config.GetTargetSysId() : 1),
#endif
    // Send to all components as we may have ex. gimbal that listens to RC instead of using Autopilot driver
    target_component_id(MAV_COMPONENT::MAV_COMP_ID_ALL)
{
}

uint32_t SerialMavlink::sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData)
{
#if defined(STARBOUND_RECEIVER)
    // Broadcast receive mode is a one-way MAVLink pipe. It must not synthesize
    // RC traffic or compete with received commands for the flight-controller UART.
    if (starboundReceiverIsBroadcastMode())
    {
        // Keep the device timeout armed while RC output is suppressed. Returning
        // DURATION_NEVER removes this callback from the scheduler permanently,
        // so switching back to Pilot mode would never resume RC overrides.
        return MAVLINK_RC_PACKET_INTERVAL;
    }
#endif
    if (!frameAvailable) {
        return DURATION_IMMEDIATELY;
    }
#if defined(STARBOUND_RECEIVER)
    pilotRcFramesReceived++;
#endif

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
    // RC, status, and relayed MAVLink frames share one serialized UART queue.
    // Writing RC directly here can interleave bytes with the main-loop drain.
    const bool queued = queueSerialBytes(buf, len);
#if defined(STARBOUND_RECEIVER)
    if (queued)
    {
        pilotRcOverridesSent++;
    }
#endif

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
#if defined(STARBOUND_RECEIVER)
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT &&
                msg.compid == MAV_COMP_ID_AUTOPILOT1 && msg.sysid != 0)
            {
                target_system_id = msg.sysid;
                target_system_learned = true;
            }
#endif
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
#if defined(STARBOUND_RECEIVER)
        if (!starboundReceiverPilotUsesMavlinkOta())
        {
            // A normal ELRS handset expects CRSF telemetry. Keep the
            // flight-controller UART purely MAVLink and translate recognized
            // ArduPilot messages before handing them to the OTA CRSF router.
            uint8_t conversionBuffer[CRSF_FRAME_NOT_COUNTED_BYTES +
                                     CRSF_PAYLOAD_SIZE_MAX]{};
            conversionBuffer[0] = CRSF_ADDRESS_USB;
            uint16_t offset = 0;
            while (offset < size)
            {
                const uint8_t count = (uint8_t)std::min(
                    (uint16_t)CRSF_PAYLOAD_SIZE_MAX,
                    (uint16_t)(size - offset));
                conversionBuffer[1] = count;
                memcpy(conversionBuffer + CRSF_FRAME_NOT_COUNTED_BYTES,
                       bytes + offset, count);
                convert_mavlink_to_crsf_telem(
                    CRSF_ADDRESS_RADIO_TRANSMITTER,
                    conversionBuffer, count);
                offset += count;
            }
            return;
        }
#endif
        mavlinkInputBuffer.atomicPushBytes(bytes, size);
    }
}

void SerialMavlink::sendQueuedData(uint32_t maxBytesToSend)
{
    const uint32_t now = millis();
#if defined(STARBOUND_RECEIVER)
    const bool broadcastMode = starboundReceiverIsBroadcastMode();
    constexpr uint32_t BROADCAST_RADIO_STATUS_INTERVAL = 1000;
    // RADIO_STATUS is state/diagnostic telemetry, not an RC transport clock.
    // Sending it at 100 Hz needlessly competes with the 100 Hz override stream
    // on the flight-controller UART and can exhaust ArduPilot's receive budget.
    const uint32_t radioStatusInterval = BROADCAST_RADIO_STATUS_INTERVAL;
    const uint16_t statusRxCounter = broadcastMode
        ? starboundReceiverHardwareErrors() : pilotRcFramesReceived;
    const uint16_t statusTxCounter = broadcastMode
        ? starboundReceiverCrcErrors() : pilotRcOverridesSent;
#else
    constexpr bool broadcastMode = false;
    constexpr uint32_t radioStatusInterval = 10;
    constexpr uint16_t statusRxCounter = 0;
    constexpr uint16_t statusTxCounter = 0;
#endif

    // Parse RF data first so received commands take priority over diagnostics.
    const uint16_t size = mavlinkOutputBuffer.size();
    if (size > 0)
    {
        uint8_t apBuf[size];
        mavlinkOutputBuffer.lock();
        mavlinkOutputBuffer.popBytes(apBuf, size);
        mavlinkOutputBuffer.unlock();

        for (uint16_t i = 0; i < size; ++i)
        {
            mavlink_message_t msg;
            mavlink_status_t status;
            if (mavlink_frame_char(MAVLINK_COMM_0, apBuf[i], &msg, &status) == MAVLINK_FRAMING_OK)
            {
                uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
                queueSerialBytes(buf, len);
            }
        }
    }

    // Both modes report status once per second. Pilot RC overrides retain their
    // independent 100 Hz cadence. Broadcast keeps the last RF-chip RSSI until
    // another valid packet arrives; sparse show traffic must not read as 0.
    if ((now - lastSentFlowCtrl) > radioStatusInterval)
    {
        lastSentFlowCtrl = now;

        // Software-based flow control for mavlink
        uint8_t percentage_remaining = ((MAV_INPUT_BUF_LEN - mavlinkInputBuffer.size()) * 100) / MAV_INPUT_BUF_LEN;
        const uint8_t broadcastRssi = linkStats.active_antenna == 0
            ? (uint8_t)linkStats.uplink_RSSI_1
            : (uint8_t)linkStats.uplink_RSSI_2;
        const uint8_t statusRssi = broadcastMode
            ? broadcastRssi
            : (uint8_t)((float)linkStats.uplink_Link_quality * 2.55);
        const uint8_t statusRemoteRssi = broadcastMode
            ? broadcastRssi
            : (uint8_t)linkStats.uplink_RSSI_1;
        const uint8_t statusNoise = broadcastMode && broadcastRssi == 0
            ? uint8_t{0}
            : (uint8_t)linkStats.uplink_SNR;

        // Populate radio status packet
        const mavlink_radio_status_t radio_status {
            // In Pilot mode these counters expose RC frames received and
            // complete MAVLink overrides written to ArduPilot. Broadcast mode
            // retains its hardware/CRC diagnostic counters.
            rxerrors: statusRxCounter,
            fixed: statusTxCounter,
            rssi: statusRssi,
            remrssi: statusRemoteRssi,
            txbuf: percentage_remaining,
            noise: statusNoise,
            // Starbound mode marker. ArduPilot consumes this only on the
            // dedicated ELRS MAVLink port; generic telemetry radios retain
            // the standard RADIO_STATUS behavior.
            remnoise: broadcastMode ? STARBOUND_RADIO_MODE_BROADCAST
                                    : STARBOUND_RADIO_MODE_PILOT,
        };

        uint8_t buf[MAVLINK_MSG_ID_RADIO_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES];
        mavlink_message_t msg;
        mavlink_msg_radio_status_encode(this_system_id, this_component_id, &msg, &radio_status);
        const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        queueSerialBytes(buf, len);
    }

#if defined(STARBOUND_RECEIVER)
    // Keep the longer diagnostic frame away from RADIO_STATUS on the UART.
    // Sending both back-to-back can overflow small downstream UART/USB FIFOs.
    constexpr bool BROADCAST_DIAGNOSTICS_ENABLED = false;
    if (BROADCAST_DIAGNOSTICS_ENABLED && broadcastMode &&
        (now - lastBroadcastDiagnostic) >= 1000 &&
        (now - lastSentFlowCtrl) >= 400)
    {
        lastBroadcastDiagnostic = now;
        char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN]{};
        if (broadcastDiagnosticPage == 0)
        {
            broadcastDiagnosticPage = 1;
            snprintf(text, sizeof(text), "SBRX P%lu S%lu/%lu H%lu/%lu",
                (unsigned long)Radio.GetStarboundPreambleCount(),
                (unsigned long)Radio.GetStarboundSyncValidCount(),
                (unsigned long)Radio.GetStarboundSyncErrorCount(),
                (unsigned long)Radio.GetStarboundHeaderValidCount(),
                (unsigned long)Radio.GetStarboundHeaderErrorCount());
        }
        else if (broadcastDiagnosticPage == 1)
        {
            broadcastDiagnosticPage = 2;
            snprintf(text, sizeof(text), "SBRX D%lu HW%u OTA%u OK%lu I%04X",
                (unsigned long)Radio.GetStarboundRxDoneCount(),
                statusRxCounter,
                statusTxCounter,
                (unsigned long)(linkStats.uplink_RSSI_1 != 0 ||
                                linkStats.uplink_RSSI_2 != 0),
                Radio.GetStarboundLastIrqStatus());
        }
        else
        {
            broadcastDiagnosticPage = 0;
            snprintf(text, sizeof(text), "SBCFG F%08lX IQ%u R%u U%02X%02X%02X%02X%02X%02X",
                (unsigned long)Radio.currFreq,
                Radio.IQinverted ? 1 : 0,
                ExpressLRS_currAirRate_Modparams->enum_rate,
                UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]);
        }

        mavlink_statustext_t statusText{};
        statusText.severity = MAV_SEVERITY_INFO;
        memcpy(statusText.text, text, sizeof(statusText.text));
        uint8_t buf[MAVLINK_MSG_ID_STATUSTEXT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES];
        mavlink_message_t msg;
        mavlink_msg_statustext_encode(this_system_id, this_component_id, &msg, &statusText);
        const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        queueSerialBytes(buf, len);
    }
#endif

    drainSerialOutput(maxBytesToSend);
}

bool SerialMavlink::queueSerialBytes(const uint8_t *data, uint16_t len)
{
    if (data == nullptr || len == 0)
    {
        return false;
    }

    mavlinkSerialOutputBuffer.lock();
    if (mavlinkSerialOutputBuffer.free() < len)
    {
        mavlinkSerialOutputBuffer.unlock();
        return false;
    }
    mavlinkSerialOutputBuffer.pushBytes(data, len);
    mavlinkSerialOutputBuffer.unlock();
    return true;
}

void SerialMavlink::drainSerialOutput(uint32_t maxBytesToSend)
{
    const int available = _outputPort->availableForWrite();
    if (available <= 0)
    {
        return;
    }

    uint8_t chunk[64];
    uint32_t budget = std::min(maxBytesToSend, (uint32_t)available);
    while (budget > 0)
    {
        mavlinkSerialOutputBuffer.lock();
        const uint16_t queued = mavlinkSerialOutputBuffer.size();
        const uint16_t count = std::min(
            (uint16_t)sizeof(chunk),
            (uint16_t)std::min(budget, (uint32_t)queued));
        if (count == 0)
        {
            mavlinkSerialOutputBuffer.unlock();
            break;
        }
        mavlinkSerialOutputBuffer.popBytes(chunk, count);
        mavlinkSerialOutputBuffer.unlock();

        // availableForWrite() reserved enough room above, so HardwareSerial
        // can enqueue this as one contiguous UART write without yielding
        // between individual MAVLink bytes.
        if (_outputPort->write(chunk, count) != count)
        {
            break;
        }
        budget -= count;
    }
}

void SerialMavlink::event()
{
#if defined(STARBOUND_RECEIVER)
    this_system_id = 255;
    if (!target_system_learned)
    {
        target_system_id = 1;
    }
#else
    this_system_id = config.GetSourceSysId() ? config.GetSourceSysId() : 255;
    target_system_id = config.GetTargetSysId() ? config.GetTargetSysId() : 1;
#endif
}

void SerialMavlink::forwardMessage(const uint8_t *data)
{
#if defined(STARBOUND_RECEIVER)
    if (starboundReceiverIsBroadcastMode())
    {
        lastBroadcastMessageReceived = millis();
        // DataUlReceiveComplete has already reassembled one complete MAVLink
        // frame. Forward it verbatim so ArduPilot dialect messages such as
        // LED_CONTROL are not rejected by ELRS's common-dialect parser.
        queueSerialBytes(data + CRSF_FRAME_NOT_COUNTED_BYTES, data[1]);
        return;
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
    mavlinkSerialOutputBuffer.flush();
#if defined(STARBOUND_RECEIVER)
    pilotRcFramesReceived = 0;
    pilotRcOverridesSent = 0;
    lastBroadcastMessageReceived = 0;
#endif
}

#endif // defined(TARGET_RX)
