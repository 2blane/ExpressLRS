Run `./build.sh` to create `elrsCombined.bin` for the RadioMaster Ranger Nano.

The build automatically selects the Ranger Nano hardware and enables the
Starbound transmit-only broadcast mode. Flash the combined image at offset
`0x0`.

The shared broadcast identity is configured by `MY_BINDING_PHRASE` in
`user_defines.txt`. The Ranger and every Starbound receiver must use the same
phrase.

## USB power command

Send a MAVLink `COMMAND_LONG` over the Ranger USB connection at 460800 baud:

- Command: `MAV_CMD_USER_2` (`31011`)
- `param1`: requested RF power in milliwatts
- Allowed values: `25`, `50`, `100`, `250`, `500`, or `1000`
- Target system: `255`
- Target component: `MAV_COMP_ID_TELEMETRY_RADIO` (`68`)

The Ranger applies and saves a valid setting, flashes white, and responds with
`COMMAND_ACK`/`MAV_RESULT_ACCEPTED`. An unsupported value returns
`MAV_RESULT_DENIED`. This command is handled locally and is not broadcast to
the receivers.
