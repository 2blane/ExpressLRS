Run ./build.sh and the firmware for Starbound Spectra will be created as elrsCombined.bin to upload to the ELRS ESP32-C3FH4.

The Starbound receiver currently starts in broadcast receive-only mode. Change
`StarboundBroadcastModeDefault` in `src/rx_main.cpp` to `false` to boot in
normal bidirectional ELRS mode instead. When its serial
protocol is MAVLink, ArduPilot can switch the volatile RF mode with a
`COMMAND_INT` or `COMMAND_LONG` message using `MAV_CMD_USER_1`:

- `MAV_CMD_USER_1` (`31010`), `param1 = 8`, `param2 = 0`: normal mode
- `MAV_CMD_USER_1` (`31010`), `param1 = 8`, `param2 = 2`: broadcast mode

Broadcast mode uses the 2.4 GHz 250 Hz LoRa profile on the initial ELRS channel,
does not bind or wait for sync, forwards received MAVLink to the flight
controller at 460800 baud, and never transmits RF. The receiver and Ranger
firmware must be built with the same binding
phrase; broadcast mode temporarily uses that compiled identity without changing
the receiver's normal binding. Rebooting restores the mode selected by
`StarboundBroadcastModeDefault`.

The binding phrase is set under user_defines.txt
