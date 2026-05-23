# CMF-06 LANC Zoom Servo

Raspberry Pi Pico 2 firmware that turns a hacked ZHIYUN CMF-06 focus/zoom motor into a LANC zoom servo.

The Pico acts as a LANC master for a zoom controller, decodes Sony LANC zoom commands, and drives the CMF-06 over UART in zoom-control mode.

## Hardware

### Pico 2 pins

| Signal | Pico pin | Notes |
|---|---:|---|
| LANC 1-wire bus | GP2 | Open-drain style PIO pin with internal pull-up |
| CMF-06 UART TX | GP0 | Connect to CMF-06 `D-` |
| CMF-06 UART RX | GP1 | Defined but not used |
| USB stdio | USB | Debug `printf` |

### CMF-06 pinout

| CMF-06 pin | Connect to |
|---|---|
| VBUS | 5V |
| GND | GND |
| D- | Pico GP0 UART TX |

The CMF-06 UART is one-way in this firmware. `D-` is treated as the CMF-06 UART RX input.

## LANC Input

The LANC side uses PIO to generate the master-side start bit and receive the controller response on the same wire.

The bus is handled as open-drain:

```text
Low:  PIO sets pin direction to output while output value is 0
High: PIO releases the pin to input/Hi-Z, internal pull-up raises the line
```

The PIO receives 8 bytes per LANC frame. Supported zoom commands are:

```text
0x28, 0x00..0x0E  Tele speed 1..8
0x28, 0x10..0x1E  Wide speed 1..8
```

All-zero frames stop zoom movement. Unknown frames are printed as `unhandled command`.

## CMF-06 UART Protocol

UART settings:

```text
115200 baud, 8 data bits, no parity, 1 stop bit
```

Packets are 14 bytes:

```text
24 3C 08 00 CC 13 00 00 00 00 VV VV CRC_L CRC_H
```

Fields:

| Offset | Size | Meaning |
|---:|---:|---|
| 0 | 2 | Header `24 3C` |
| 2 | 2 | Payload length `08 00` |
| 4 | 1 | Command |
| 5 | 1 | Fixed `13` |
| 6 | 4 | Fixed `00 00 00 00` |
| 10 | 2 | Position value, little endian |
| 12 | 2 | CRC-16/XMODEM, little endian |

Commands:

```text
0xB8  Focus packet
0xC8  Zoom packet
```

CRC is calculated over the 8-byte payload only:

```text
CC 13 00 00 00 00 VV VV
```

CRC parameters:

```text
CRC-16/XMODEM
poly   = 0x1021
init   = 0x0000
xorout = 0x0000
refin  = false
refout = false
```

The firmware continuously alternates focus and zoom packets on core1:

```text
Focus packet
18.8ms wait
Zoom packet
18.8ms wait
```

Focus is held at position `0`. Zoom starts at `0` and moves by adding or subtracting a speed-dependent step while a LANC zoom command is present. When the command disappears, the last position is held.

## Configuration

Main options are near the top of [main.c](./main.c):

```c
#define FOLLOW_FOCUS_REVERSE_ZOOM true
#define FOLLOW_FOCUS_WRAP_ZOOM false
#define FOLLOW_FOCUS_END_DECEL_COUNTS 512
#define ZOOM_RAMP_COMMANDS 10
```

`FOLLOW_FOCUS_REVERSE_ZOOM` swaps Tele/Wide direction.

`FOLLOW_FOCUS_WRAP_ZOOM` controls endpoint behavior:

```text
true:  0xFFFF wraps to 0x0000, and 0x0000 wraps to 0xFFFF
false: endpoints clamp, with deceleration near the final 512 counts
```

Zoom speed map:

```text
speed 1 -> 50
speed 2 -> 100
speed 3 -> 200
speed 4 -> 400
speed 5 -> 750
speed 6 -> 1250
speed 7 -> 2000
speed 8 -> 3000
```

## Build

Configure and build with the Pico SDK environment:

```powershell
cmake -G Ninja -S . -B build-ninja -DPython3_EXECUTABLE="C:/Program Files/KiCad/10.0/bin/python.exe" -DCMAKE_MAKE_PROGRAM="C:/Users/admin/.pico-sdk/ninja/v1.12.1/ninja.exe"
cmake --build build-ninja
```

The UF2 output is:

```text
build-ninja/cmf06_lanc_zoom_servo.uf2
```
