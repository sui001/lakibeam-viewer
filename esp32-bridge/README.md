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
| W5500 SPI module | Prefer the small "Lite" board, see below. |
| LakiBeam 1 | 12V on the barrel jack, Ethernet. There is no USB interface. |

### Which W5500 module

Two are commonly sold. **Prefer the small one**, roughly 23 x 28.5mm with the
RJ45 taking up most of the board.

The larger board, around 55 x 28mm, is bigger because it carries a 3.3V
regulator and 5V tolerant buffering so it can be driven from a 5V Arduino Uno
or Mega. An ESP32 is natively 3.3V, so that circuitry solves a problem you do
not have and costs you twice the length.

Two things to check on the small module:

- **Does it break out `INT` and `RST`?** Many Lite boards only give you MOSI,
  MISO, SCK, SCS, 3V3 and GND. That is fine. Set `ETH_IRQ` and `ETH_RST` to
  `-1` in the sketch and the driver polls instead of using the interrupt.
- **Power.** The W5500 draws roughly 130 to 180mA with the link up, on top of
  the ESP32's own peaks. Usually fine from the SuperMini's 3V3 pin. If the link
  drops under traffic or the board resets, give the W5500 its own 3.3V supply.

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
| INT | GPIO8 | set `ETH_IRQ` to -1 if your module lacks it |
| RST | GPIO9 | set `ETH_RST` to -1 if your module lacks it |
| 3V3 | 3V3 | **not 5V**, the module is 3.3V logic |
| GND | GND | |

Flight controller serial:

| Bridge | Flight controller |
|---|---|
| GPIO5 (TX) | serial RX |
| GPIO4 (RX) | serial TX |
| GND | GND |

Ground between the bridge and the flight controller is not optional.

Every pin here is within **GPIO 1 to 13**. That is a hard limit on the
SuperMini: pinouts online list 32 GPIO and the board really does route them,
but only 1 to 13 reach the headers. The rest are bare pads needing pins
soldered on. GPIO 2 is skipped as a strapping pin, and 10 to 13 are the S3's
native FSPI block, which is why they carry SPI.

Change them at the top of the sketch if your board differs, but check against
your board's pinout first.

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

That works in the Arduino IDE. **It does not work with `arduino-cli`**, whose
library resolver matches on a header at the library root and so never finds
`mavlink.h` one folder down in `common/`. Passing `--libraries` at the parent
directory does not help either. Point the compiler at it instead:

```
arduino-cli compile   --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashSize=4M,PartitionScheme=default"   --build-property "compiler.cpp.extra_flags=-I/path/to/c_library_v2 -Wno-address-of-packed-member"   lakibeam_bridge
```

Built that way the sketch is about 59% of flash and 10% of RAM on a 4MB S3.

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

**The bridge configures the sensor itself now**, five seconds after the link
comes up and only if no packets have arrived. You should not have to do
anything. This section is kept because it explains what the bridge is doing and
how to do it by hand if that fails.

The LakiBeam ships with its laser and rotor both off, and enabling the laser
alone leaves the rotor at 0 rpm and produces no points at all, which looks
exactly like dead hardware.

The vendor documentation says these settings persist to the sensor's EEPROM.
**Measured on 2026-09-09 they did not survive a power cycle**: a sensor that was
enabled and streaming came back silent after its 12V was cut, and needed the
enable sent again. Treat the enable as something that has to happen on every
boot, which is why the bridge does it.

To do it by hand, run the Python viewer in the parent directory once, on any
machine:

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

On a Cube Orange+ the mapping is **TELEM1 = SERIAL1, TELEM2 = SERIAL2**,
confirmed against ArduPilot's `hwdef/CubeOrangePlus/README.md`. Prefer
TELEM2 and leave TELEM1 for a telemetry radio.

RTS/CTS can be left unwired. It gates the autopilot's transmitting, not its
receiving, so it is not in the path of an inbound sensor feed.

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

~~No configuration of the sensor.~~ Fixed. `lakibeam_bringup` now enables the
laser and rotor itself over HTTP, five seconds after the link comes up, and
only if no packets have arrived. It is guarded on packet count rather than run
at every boot because the settings persist to EEPROM, so a sensor that is
already scanning needs no write.

**No health reporting.** The bridge does not tell the flight controller when
the link drops. If the sensor dies, sectors go to unknown and ArduPilot simply
sees no obstacles, which is the dangerous failure direction. Worth adding a
`DISTANCE_SENSOR` health heartbeat before relying on this outdoors.

~~The MAVLink path is untested.~~ **The whole chain works.** Confirmed on
2026-09-12 against a real LakiBeam, an ESP32-S3 SuperMini and a Cube Orange+
on TELEM2: 178 packets a second in, 176 points per packet, 10Hz of
`OBSTACLE_DISTANCE` out, and ArduPilot republishing it as proximity with 44 of
72 sectors carrying returns from 20cm to 11.9m. `increment_f` arrived as 5.00
and the limits as 20/1500cm, matching what the bridge packs, so nothing is
reinterpreted in transit.

Two parameters are what stand between "wired correctly" and "working", and
both fail silently: `PRX1_TYPE` defaults to 0, which makes ArduPilot discard
`OBSTACLE_DISTANCE` on arrival, and `SERIAL2_BAUD` defaults to 57, so the port
listens at 57600 while the bridge sends at 921600. Neither produces an error.

## Bring-up: prove the hardware before the MAVLink

`lakibeam_bringup/` is a second sketch that exercises everything except
MAVLink: SPI to the W5500, the Ethernet link, UDP from the sensor and the
packet decode. It needs no MAVLink library and nothing connected to the flight
controller, so it is the right thing to flash first.

It reports every two seconds over USB serial, and each report restates whether
the W5500 is answering rather than saying so once at boot, because a boot
banner has scrolled away by the time anyone opens a monitor.

Read it in this order:

| Line | Meaning |
|---|---|
| `w5500 NOT RESPONDING over SPI` | SPI wiring or power. Nothing else matters until this clears. |
| `w5500 responding over SPI` + `link DOWN` | SPI is proven. No Ethernet link: sensor unpowered, or cable. |
| `link UP` + `packets 0` | Cable and sensor fine. Laser and rotor are almost certainly off. |
| `packets` with sensible distances | The hardware half is done. Move to `lakibeam_bridge`. |

## Testing without a flight controller

The bridge prints statistics to USB serial every 5 seconds:

```
[stat] packets in 1074, mavlink out 50, link up
```

Roughly 1000 packets in per 5 seconds and 50 MAVLink frames out is correct at
10Hz. Zero packets in with the link up almost always means the laser or rotor
was never enabled.
