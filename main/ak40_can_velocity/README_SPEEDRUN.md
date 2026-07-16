# AK40 CAN Velocity Speedrun

Quick reference for the `arduino_uno/ak40_can_velocity` sketch.

## Hardware

- Arduino Uno R4
- Joy-IT `SBC-CAN01` / MCP2515 CAN module
- CubeMars `AK40-10`
- External motor power supply

## Wiring

### Uno R4 -> MCP2515

| Uno R4 | MCP2515 / SBC-CAN01 |
|---|---|
| `5V` | `VCC` |
| `5V` | `VCC1` |
| `GND` | `GND` |
| `D10` | `CS` |
| `D11` | `SI` / `MOSI` |
| `D12` | `SO` / `MISO` |
| `D13` | `SCK` |
| `D2` | `INT` |

### MCP2515 -> AK40

| MCP2515 | AK40 |
|---|---|
| `CANH` | `CAN_H` white |
| `CANL` | `CAN_L` blue |

### Important

- Enable the `120 ohm` termination jumper on the CAN module if this is a 2-node bus.
- Power the motor from its own supply, not from the Arduino.
- The sketch keeps:
  - motor ID `69`
  - servo CAN handshake
  - existing status parsing

## Files

- `main.ino` - application loop
- `MotorCAN.*` - MCP2515 + AK40 servo CAN transport
- `MotionController.*` - ramped speed control state machine
- `SerialCommand.*` - serial command parser
- `Status.*` - parsed telemetry and status printing
- `Config.h` - pinout and tuning constants

## Arduino IDE

- Board: `Arduino Uno R4`
- Library: `mcp2515` by `autowp`
- Serial monitor: `115200 baud`

## Supported Serial Commands

| Command | Action |
|---|---|
| `v <erpm>` | Set target speed |
| `s` | Smooth stop |
| `e` | Emergency stop and release motor passive |
| `i` | Print status |
| `h` | Reconnect CAN |

Examples:

```text
v 800
v -600
s
e
i
h
```

## Runtime Behavior

Main loop order:

1. Poll CAN
2. Read serial
3. Update motion controller
4. Send current ERPM to motor
5. Print status

The motion controller ramps speed every `20 ms` using:

- `ACCEL_STEP`
- `STOP_STEP`
- `EMERGENCY_STEP`

States:

- `IDLE`
- `MOVING_UP`
- `MOVING_DOWN`
- `STOPPING`
- `EMERGENCY_STOP`

## Status Output

The sketch prints:

- motion state
- target ERPM
- current ERPM
- measured ERPM
- position
- temperature
- error
- current
- estimated torque

Torque is an estimate derived from CAN current telemetry. It is not a direct torque measurement from the motor.

## Startup

On boot the sketch:

1. Starts serial
2. Initializes MCP2515
3. Runs servo CAN connect / handshake
4. Waits for commands unless `AUTO_START` is enabled in `Config.h`

## Useful Config Values

See `Config.h`:

- `kDefaultErpm`
- `kMaxErpm`
- `kAutoStart`
- `kCommandPeriodMs`
- `kStatusPeriodMs`
- `kMotionUpdatePeriodMs`
- `kAccelStep`
- `kStopStep`
- `kEmergencyStep`

## Safety

- Start with low ERPM values.
- Keep the shaft unloaded for first tests.
- Use `s` before disconnecting power.
- If `err != 0`, the controller enters `EMERGENCY_STOP`.
