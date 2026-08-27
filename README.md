# LakiBeam viewer

Live room scanning for the Richbeam LakiBeam 1 LIDAR. Reads the sensor's UDP
point stream and draws the room in a browser.

Runs on Windows, macOS and Linux. Flask is the only dependency.

![no screenshot yet](https://img.shields.io/badge/status-working-brightgreen)

## Install

```
pip install -r requirements.txt
```

## Run

```
python lakibeam_server.py
```

Then open <http://localhost:5002>.

Options:

```
--lidar-ip 192.168.198.2   sensor address
--udp-port 2368            port the sensor sends to
--web-port 5002            port to serve the viewer on
--no-configure             skip laser/rotor setup, just listen
```

## Wiring and network

The sensor needs **12V on its barrel jack** and **Ethernet** to the host
machine. There is no USB interface: a USB cable alone will never bring it up.

By default the sensor is at `192.168.198.2` and expects the host at
`192.168.198.1`. Give your machine a static address on that subnet:

- **Windows** - Network adapter properties, IPv4, static IP `192.168.198.1`,
  mask `255.255.255.0`, no gateway.
- **Linux** - `sudo ip addr add 192.168.198.1/24 dev eth0`

Confirm the sensor is reachable before anything else:

```
ping 192.168.198.2
```

## Things that cost us time

Worth reading before debugging. Each of these looks like a broken sensor.

**The laser and rotor are both off by default, and the setting persists.**
Enabling the laser alone is not enough - the rotor stays at 0 rpm and no
points are produced. Both are needed:

```
method=-en 1      laser on
method=-freq 5    rotor at fastest scan rate
```

These are written to the sensor's EEPROM, so whatever state it was left in
survives a power cycle. A unit that arrives "dead" may simply have been
switched off in software by whoever used it last. This server sets both on
startup.

**The config API needs a Referer header.** Without it the request hangs until
it times out instead of returning an error:

```
POST http://192.168.198.2/cgi-bin/config.php
Referer: http://192.168.198.2/config.html
```

**The web panel becomes unusable while the sensor is scanning.** Its control
plane is starved by the scan workload: HTTP succeeds roughly one attempt in
three and takes several seconds, and ICMP is mostly dropped. Since the panel
pulls about fifteen assets, it effectively never finishes loading in a
browser. This is normal behaviour, not a fault, and not a network problem.
Stop the laser first if you need the panel, or put a retrying proxy in front
of it.

**The panel has no point cloud viewer.** It offers Dashboard, LiDAR
Configuration and Firmware Update only. The Dashboard graphs are system
health - voltage, temperature, load, motor rpm, laser state - not scan data.
That is the reason this project exists.

**`resolution: 4.00` in telemetry means "laser stopped".** It is a sentinel
value, not a real angular resolution. The vendor panel displays it as
"Laser stopped." Do not read it as a coarse resolution setting.

**Scan window is 45 to 315 degrees.** A 270 degree field of view with a
90 degree blind spot behind the sensor. Points outside that range are not a
decoding bug.

## Packet format

1206 bytes per UDP packet:

```
12 x sub_packet (100 bytes) + uint32 timestamp + uint16 factory

sub_packet:
    uint16  header      0xEEFF
    uint16  azimuth     hundredths of a degree
    16 x    point       uint16 dist_mm, uint8 rssi, uint16 dist_mm, uint8 rssi
```

`dist == 0` means no return. `azimuth == 0xFFFF` marks an invalid sub-packet.

Azimuth is reported once per sub-packet, so the sixteen points within it are
interpolated between that azimuth and the next one. Getting this wrong is a
common source of maps that look compressed or smeared.

## Notes on the view

The sensor is assumed to be **stationary**. Points are drawn in the sensor's
own frame from a fixed origin, with no pose estimation anywhere in the path,
so the picture cannot drift.

Hits accumulate into 50mm cells and are never evicted. A fixed sensor sees a
bounded room, so the cell count converges instead of growing without limit.
Evicting cells from an accumulating map is what makes it churn and smear, and
that reads as drift even when the origin is perfectly fixed.

If you mount this on something that moves, you need pose estimation and this
approach no longer applies.

## Licence

MIT
