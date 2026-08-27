# ESP32 bridge: LakiBeam to ArduPilot

Puts a LakiBeam 1 on a flight controller as a proximity sensor. The bridge reads
the sensor's UDP stream over Ethernet, reduces it to 72 sectors, and sends
MAVLink `OBSTACLE_DISTANCE` over serial. ArduPilot then treats it as a native
360 degree proximity sensor, the same as an RPLidar or a LightWare SF40C.

Target hardware is an **ESP32-S3 SuperMini** and a **W5500** Ethernet module,
talking to a **Cube Orange+**, but nothing here is specific to that flight
controller.

## Why not Ethernet straight into the Cube, or CAN

Both were considered first. Neither works well.

**CAN cannot carry the raw stream.** The sensor produces roughly **2 Mbit/s**
(around 215 packets a second at 1206 bytes). Classic CAN tops out at 1 Mbit/s
theoretical and realistically delivers about half that after framing and
arbitration overhead, so it is about four times too slow. Telling detail:
CubePilot's own `CubeNode ETH` adapter solves this problem by moving data over
**UART at 12.5 Mbaud using PPP**, and uses CAN only for configuration. They hit
the same wall.

**Ethernet into the Cube does not help by itself.** ArduPilot 4.5+ can treat a
UDP stream as a serial device, and `CubeNode ETH` will give a Cube Orange+ an
Ethernet port. But ArduPilot has **no LakiBeam driver**. Its supported 360
degree lidars are the LightWare SF40C, TeraRanger Tower and RPLidar. Plumbing
the packets through unchanged just delivers a format nothing understands.

**The data does not need to be raw.** `OBSTACLE_DISTANCE` carries 72 `uint16`
values, roughly **11 kbit/s at 10Hz**. The useful work is parsing and reducing
2 Mbit/s down to that, which is what this bridge does. Once reduced, the
transport is almost irrelevant.

A custom Ethernet-to-CAN PCB would mean an Ethernet PHY and magnetics, an MCU
with a MAC, a TCP/IP stack, a CAN controller and transceiver, DroneCAN firmware,
and still writing the ArduPilot side. Weeks of work to arrive in the same place
as a $5 module.

## Parts

| Part | Notes |
|---|---|
| ESP32-S3 SuperMini | Any ESP32 works. The S3 has no Ethernet MAC, hence the W5500. |
| W5500 SPI module | The common blue breakout is fine. Needs 3.3V logic. |
| LakiBeam 1 | 12V on the barrel jack, Ethernet. There is no USB interface. |

An original ESP32 with a LAN8720 would be faster, since it has a real Ethernet
MAC and uses RMII rather than SPI. It is not necessary: W5500 over SPI manages
8 to 15 Mbit/s against a 2 Mbit/s stream.

## Wiring

W5500 to ESP32-S3 SuperMini:

| W5500 | ESP32-S3 | Note |
|---|---|---|
| SCK | GPIO12 | |
| MISO | GPIO13 | |
| MOSI | GPIO11 | |
| SCS | GPIO10 | chip select |
| INT | GPIO14 | |
| RST | GPIO9 | |
| 3V3 | 3V3 | **not 5V**, the module is 3.3V logic |
| GND | GND | |

Flight controller serial:

| Bridge | Flight controller |
|---|---|
| GPIO17 (TX) | serial RX |
| GPIO18 (RX) | serial TX |
| GND | GND |

Ground between the bridge and the flight controller is not optional.

These pins avoid the SuperMini's strapping pins (GPIO0, 2, 45, 46) and the
onboard RGB LED on GPIO47. Change them at the top of the sketch if your board
differs, but check against your board's pinout first.

## Build

**Arduino ESP32 core 3.x or later.** W5500 support is built into `ETH.h` from
core 3.0. On core 2.x it does not exist and this will not compile.

**MAVLink headers.** Not included here, they are large and generated. Download
the C library and put it where the sketch can see it:

```
git clone https://github.com/mavlink/c_library_v2
```

Then either copy `c_library_v2` into your Arduino `libraries` folder, or drop
its contents beside the `.ino`. The sketch includes `<common/mavlink.h>`.

**Board settings** for the SuperMini:

| Setting | Value |
|---|---|
| Board | ESP32S3 Dev Module |
| USB CDC On Boot | Enabled |
| Flash Size | 4MB |
| Partition Scheme | Default 4MB with spiffs |

Without USB CDC On Boot enabled the sketch runs but the serial monitor stays
blank.

## Enable the sensor first

**The bridge does not configure the sensor.** The LakiBeam ships with its laser
and rotor both off, and enabling the laser alone leaves the rotor at 0 rpm and
produces no points at all, which looks exactly like dead hardware.

Those settings **persist to the sensor's EEPROM**, so this is a one time job.
Run the Python viewer in the parent directory once, on any machine:

```
python ../lakibeam_server.py
```

It sends `-en 1` and `-freq 5` on startup. After that the sensor comes up
scanning on its own and the bridge will see data immediately.

You can also do it by hand:

```
curl -X POST -H "Referer: http://192.168.198.2/config.html" \
     --data "method=-freq 5" http://192.168.198.2/cgi-bin/config.php
curl -X POST -H "Referer: http://192.168.198.2/config.html" \
     --data "method=-en 1"  http://192.168.198.2/cgi-bin/config.php
```

The `Referer` header is required. Without it the request hangs until it times
out rather than returning an error.

## ArduPilot parameters

| Parameter | Value | Meaning |
|---|---|---|
| `PRX1_TYPE` | `2` | MAVLink proximity source |
| `SERIALn_PROTOCOL` | `2` | MAVLink2 on the port you wired to |
| `SERIALn_BAUD` | `921` | 921600, matching `FC_BAUD` in the sketch |

Replace `n` with the serial port number you used. On a Cube Orange+, Serial1 or
Serial2 are the usual choices.

Check it is working in Mission Planner's proximity view, or with MAVProxy:

```
module load proximity
```

You should see the room appear as a ring of returns.

## Scan geometry

The sensor's window is **45 to 315 degrees**: a 270 degree field of view with a
90 degree blind spot behind it. Sectors in that blind arc are reported as
`UINT16_MAX`, which is MAVLink's "no reading" value, rather than as zero. Zero
would read as an obstacle at zero distance and trigger avoidance against
nothing.

If the sensor is not mounted facing forward, set `MOUNT_YAW_DEG` at the top of
the sketch rather than rotating anything in ArduPilot. Sector 0 must correspond
to straight ahead.

Each sector keeps the **nearest** return, not an average. For avoidance the
closest thing in a sector is the one that matters, and averaging would let a
near object be hidden by open space beside it.

## Known limitations

**No configuration of the sensor.** See above. One time setup, persists to
EEPROM.

**No health reporting.** The bridge does not tell the flight controller when
the link drops. If the sensor dies, sectors go to unknown and ArduPilot simply
sees no obstacles, which is the dangerous failure direction. Worth adding a
`DISTANCE_SENSOR` health heartbeat before relying on this outdoors.

**Untested against hardware.** The decoder logic is a direct port of the Python
in the parent directory, which is confirmed working against a real sensor. The
MAVLink and Ethernet paths have not yet been run on a flight controller.

## Testing without a flight controller

The bridge prints statistics to USB serial every 5 seconds:

```
[stat] packets in 1074, mavlink out 50, link up
```

Roughly 1000 packets in per 5 seconds and 50 MAVLink frames out is correct at
10Hz. Zero packets in with the link up almost always means the laser or rotor
was never enabled.
