/*
 * LakiBeam 1 -> ArduPilot proximity bridge
 *
 * Reads the LIDAR's UDP point stream over Ethernet, collapses it into 72
 * five-degree sectors, and sends MAVLink OBSTACLE_DISTANCE to a flight
 * controller over serial. ArduPilot treats it as a native proximity sensor.
 *
 * Hardware: ESP32-S3 SuperMini + W5500 Ethernet module.
 * See ../README.md for wiring, MAVLink library setup and ArduPilot parameters.
 *
 * Why not send the raw scan? The sensor produces roughly 2 Mbit/s. Classic CAN
 * tops out at 1 Mbit/s and realistically delivers half that, so the raw stream
 * does not fit. It does not need to: OBSTACLE_DISTANCE is 72 uint16 values,
 * about 11 kbit/s at 10Hz. The reduction happens here, on the bridge.
 */

#include <ETH.h>
#include <SPI.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <common/mavlink.h>

// ── Wiring ───────────────────────────────────────────────────────────────────
// W5500 on SPI. Every pin here is within GPIO 1-13, which is all the SuperMini
// breaks out to headers. Anything above 13 exists on the chip but would need
// pins soldered to bare pads. GPIO 2 is avoided as a strapping pin.
// 10-13 are the S3's native FSPI block, which is why they are the SPI four.
#define ETH_SCK    12
#define ETH_MISO   13
#define ETH_MOSI   11
#define ETH_CS     10

// INT and RST. The small "W5500 Lite" module does break both out, so use them:
// you get the interrupt rather than polling, and a real hardware reset line.
// On a module that omits them, set either to -1 and the driver copes.
#define ETH_IRQ     8      // -1 if not available
#define ETH_RST     9      // -1 if not available

// Serial link to the flight controller. Goes to a TELEM port (6-pin JST-GH),
// not CAN: FC_TX to TELEM pin 3 (RX), FC_RX to TELEM pin 2 (TX), GND to pin 6.
//
// The headers split 1-7 on one side and 8-13 on the other, so these two sit
// well clear of the SPI block. GPIO 2 is skipped as the strapping pin this
// board misbehaves on.
#define FC_TX       5
#define FC_RX       4
#define FC_BAUD    921600

// ── Network ──────────────────────────────────────────────────────────────────
// The sensor ships at 192.168.198.2 and sends to 192.168.198.1:2368.
static const IPAddress LOCAL_IP (192, 168, 198, 1);
static const IPAddress NETMASK  (255, 255, 255, 0);
static const IPAddress GATEWAY  (192, 168, 198, 1);
static const IPAddress LIDAR_IP (192, 168, 198, 2);
static const uint16_t  SCAN_PORT = 2368;

// ── Scan geometry ────────────────────────────────────────────────────────────
#define NUM_SECTORS   72            // MAVLink OBSTACLE_DISTANCE maximum
#define SECTOR_DEG    (360.0f / NUM_SECTORS)   // 5 degrees

#define MIN_DIST_CM   20
#define MAX_DIST_CM   1500          // 15m, beyond the sensor's useful range
#define DIST_UNKNOWN  UINT16_MAX    // MAVLink "no reading" value

// Rotate the scan if the sensor is not mounted facing forward. Positive values
// rotate the picture clockwise. Sector 0 is straight ahead after this is applied.
//
// 180 because the LakiBeam's 90 degree blind arc sits at 315 to 45 degrees in
// its own frame, which is behind the sensor. Left at 0, that blind arc lands on
// MAVLink sector 0, so the vehicle is blind straight ahead and fully sighted
// behind it. Exactly backwards.
//
// The tell, if this is ever wrong again: ArduPilot publishes DISTANCE_SENSOR
// for orientations 1 to 7 but never 0. A missing sector 0 means the blind arc
// is pointing forward.
#define MOUNT_YAW_DEG 180.0f

// The LakiBeam's azimuth increases the opposite way round to MAVLink's sector
// numbering, so mapping one straight onto the other produces a mirror image:
// an obstacle on the left is reported on the right. Found the hard way, by
// having to physically invert the sensor to get the correct side, which works
// precisely because it reverses the direction of rotation.
//
// Negating the angle does the same thing in software. The blind arc is
// symmetric about the sensor's zero, so this does not move it and
// MOUNT_YAW_DEG stays 180.
//
// Set to 0 if a future sensor or firmware numbers its azimuth the other way.
// The tell is that the picture looks plausible but turns the wrong way when
// you walk around the machine.
#define MOUNT_MIRROR 1

// ── Packet format ────────────────────────────────────────────────────────────
// 1206 bytes: 12 sub-packets of 100 bytes, then uint32 timestamp + uint16 factory.
// sub-packet: uint16 header (0xEEFF), uint16 azimuth (0.01 deg),
//             then 16 points of (uint16 dist_mm, uint8 rssi) x2.
#define PKT_LEN       1206
#define BLOCKS        12
#define BLOCK_POINTS  16
#define BLOCK_SIZE    100
#define AZ_INVALID    0xFFFF

// ── Output rate ──────────────────────────────────────────────────────────────
#define SEND_INTERVAL_MS 100        // 10Hz

// Go quiet if the sensor has said nothing for this long. Comfortably longer
// than the ~6ms between packets at 179/sec, so jitter never trips it, and well
// under ArduPilot's own proximity timeout so the autopilot notices promptly.
#define SENSOR_STALE_MS 500

// How long to wait after link-up before deciding the sensor is silent because
// its laser and rotor are off, rather than because it is still starting.
#define CONFIG_AFTER_MS 5000

// ── Status LED ───────────────────────────────────────────────────────────────
// Same ladder as lakibeam_bringup, deliberately. Once this is on a machine
// there is no serial monitor, and the light is the only thing that will tell
// you why the proximity ring is empty.
//
// Pin varies by vendor on these boards: 48 here, 47 on some others.
// led_pin_test/ settles it in one flash. neopixelWrite is built into ESP32
// core 3.x, no library needed.
#define LED_PIN        48
#define LED_LEVEL      40      // of 255. Full brightness is unpleasant to sit beside.
#define FLASH_FAST_MS  120
#define FLASH_SLOW_MS  500
#define STALE_MS       1000    // no packet for this long counts as silent

/*
 * There is no separate state for "MAVLink going out", because that is always
 * true: send_obstacle_distance() fires every 100ms whether or not a single
 * packet arrived. A light wired to that would be solid green while ArduPilot
 * is being told the world is empty, which is the exact failure this project
 * calls the dangerous direction.
 *
 * So steady green is gated on returns that actually landed in a sector, which
 * is the same thing the autopilot will see.
 */
enum Status {
  ST_FAULT,      // red, fast     W5500 not answering. Solder or power.
  ST_NOLINK,     // red, slow     No Ethernet link. Sensor unpowered, or cable.
  ST_SILENT,     // amber, slow   Link up, sensor not scanning.
  ST_DATA,       // green, slow   Packets arriving, but nothing landing in a sector.
  ST_RUNNING     // green, steady Sectors carrying real returns.
};

// MAVLink identity of this bridge. Must not collide with the autopilot (1).
#define MAV_SYS_ID   1
#define MAV_COMP_ID  MAV_COMP_ID_OBSTACLE_AVOIDANCE

// ── State ────────────────────────────────────────────────────────────────────
static WiFiUDP  udp;
static uint8_t  pkt[PKT_LEN + 64];
static uint16_t sectors[NUM_SECTORS];
static bool     eth_up = false;
static uint32_t last_send_ms = 0;
static uint32_t packets_in = 0, frames_out = 0;
static uint32_t last_stat_ms = 0;
static bool     config_done = false;   // laser/rotor enable already attempted
static uint32_t link_up_ms = 0;
static uint32_t packets_total = 0;     // survives the stat window reset
static bool     eth_init_ok = false;   // did the W5500 answer over SPI at all
static uint32_t last_packet_ms = 0;
static uint32_t last_return_ms = 0;    // last packet that put a return in a sector
static bool     saw_return = false;    // set by sector_add for the packet in hand
static bool     was_alive = true;      // for logging health transitions only
static uint32_t frames_muted = 0;      // frames deliberately not sent

static void sectors_reset() {
  for (int i = 0; i < NUM_SECTORS; i++) sectors[i] = DIST_UNKNOWN;
}

/*
 * Fold one reading into its sector, keeping the nearest.
 *
 * Nearest rather than average: for obstacle avoidance the closest thing in a
 * sector is the one that matters. Averaging would let a near object be masked
 * by open space either side of it.
 */
static inline void sector_add(float angle_deg, uint16_t dist_mm) {
  if (dist_mm == 0) return;

  uint16_t cm = dist_mm / 10;
  if (cm < MIN_DIST_CM || cm > MAX_DIST_CM) return;

  float a = (MOUNT_MIRROR ? -angle_deg : angle_deg) + MOUNT_YAW_DEG;
  a = fmodf(a, 360.0f);
  if (a < 0) a += 360.0f;

  int idx = (int)(a / SECTOR_DEG);
  if (idx < 0 || idx >= NUM_SECTORS) return;

  // Set here rather than on any non-zero reading, so the status light reflects
  // what actually reaches the autopilot, not what merely arrived on the wire.
  saw_return = true;

  if (sectors[idx] == DIST_UNKNOWN || cm < sectors[idx]) sectors[idx] = cm;
}

/*
 * Decode one UDP packet into sectors.
 *
 * Azimuth is reported once per sub-packet, so the 16 points inside it are
 * interpolated between this azimuth and the next one. Skipping that step is a
 * common source of maps that look compressed or smeared.
 */
static void decode_packet(const uint8_t *p, int len) {
  if (len < PKT_LEN) return;

  for (int b = 0; b < BLOCKS - 1; b++) {
    const uint8_t *blk = p + b * BLOCK_SIZE;

    uint16_t az = (uint16_t)(blk[2] | (blk[3] << 8));
    if (az == AZ_INVALID) continue;

    uint16_t az_next = (uint16_t)(p[(b + 1) * BLOCK_SIZE + 2] |
                                  (p[(b + 1) * BLOCK_SIZE + 3] << 8));
    if (az_next == AZ_INVALID) az_next = az;

    az      %= 36000;
    az_next %= 36000;

    // Wrap through 360 correctly when the block straddles zero.
    int32_t span = (int32_t)az_next - (int32_t)az;
    if (span < 0) span += 36000;
    float step = (float)span / BLOCK_POINTS;

    for (int j = 0; j < BLOCK_POINTS; j++) {
      const uint8_t *pt = blk + 4 + j * 6;
      uint16_t dist_mm = (uint16_t)(pt[0] | (pt[1] << 8));
      float a = fmodf((az + j * step) * 0.01f, 360.0f);
      sector_add(a, dist_mm);
    }
  }
}

static void send_obstacle_distance() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_obstacle_distance_pack(
      MAV_SYS_ID, MAV_COMP_ID, &msg,
      (uint64_t)micros(),
      MAV_DISTANCE_SENSOR_LASER,
      sectors,
      0,                       // increment (deg, uint8) - 0 means use increment_f
      MIN_DIST_CM,
      MAX_DIST_CM,
      SECTOR_DEG,              // increment_f, the one ArduPilot actually reads
      0.0f,                    // angle_offset: sector 0 is already forward
      MAV_FRAME_BODY_FRD);

  uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, n);
  frames_out++;
}

/*
 * Turn the laser and the rotor on.
 *
 * They are separate and both off by default. Enabling the laser alone leaves
 * rpm at 0 and produces no points at all, which looks exactly like dead
 * hardware.
 *
 * This is not a convenience. The vendor documentation says these settings
 * persist to EEPROM, but measured on 2026-09-09 they did not survive a power
 * cycle: a sensor enabled and streaming came back silent after its 12V was
 * cut. Without this the bridge would come up on every boot happily sending
 * ArduPilot a proximity ring with nothing in it, which reads as "no obstacles"
 * rather than as a fault. That is the dangerous direction to fail in.
 *
 * Two things make the request fussy. The CGI needs a Referer header or it
 * hangs until timeout instead of returning an error. And the sensor's control
 * plane starves once it is scanning, so this only runs while it is still
 * quiet, which is the only time it is reliable anyway.
 */
static Status status_now() {
  if (!eth_init_ok) return ST_FAULT;
  if (!eth_up)      return ST_NOLINK;

  uint32_t now = millis();
  if (now - last_packet_ms > STALE_MS) return ST_SILENT;
  if (now - last_return_ms > STALE_MS) return ST_DATA;
  return ST_RUNNING;
}

static void led_update() {
  static const char *name[] = { "FAULT", "NO LINK", "SILENT", "DATA", "RUNNING" };
  static uint32_t phase_ms = 0;
  static bool     lit = false;
  static Status   shown = ST_RUNNING;   // forces an announce on the first pass
  static bool     announced = false;

  Status   st = status_now();
  uint32_t now = millis();
  uint8_t  r = 0, g = 0;
  uint32_t period;

  switch (st) {
    case ST_FAULT:  r = LED_LEVEL;                    period = FLASH_FAST_MS; break;
    case ST_NOLINK: r = LED_LEVEL;                    period = FLASH_SLOW_MS; break;
    case ST_SILENT: r = LED_LEVEL; g = LED_LEVEL / 3; period = FLASH_SLOW_MS; break;
    case ST_DATA:                  g = LED_LEVEL;     period = FLASH_SLOW_MS; break;
    default:                       g = LED_LEVEL;     period = 0;             break;
  }

  // Announce transitions so the serial log and the LED can never disagree
  // about what the bridge thinks its state is.
  if (st != shown || !announced) {
    Serial.printf("[led] %s\n", name[st]);
    shown = st;
    announced = true;
    lit = true;
    phase_ms = now;
    neopixelWrite(LED_PIN, r, g, 0);
    return;
  }

  if (period == 0) {
    if (!lit) { lit = true; neopixelWrite(LED_PIN, r, g, 0); }
    return;
  }

  if (now - phase_ms >= period) {
    phase_ms = now;
    lit = !lit;
    if (lit) neopixelWrite(LED_PIN, r, g, 0);
    else     neopixelWrite(LED_PIN, 0, 0, 0);
  }
}

static bool sensor_configure() {
  // Rotor first. Enabling the laser against a stopped rotor is the failure
  // mode this whole function exists to avoid.
  const char *cmds[] = { "method=-freq 5", "method=-en 1" };
  const char *what[] = { "rotor to fastest scan rate", "laser on" };

  char url[64];
  snprintf(url, sizeof(url), "http://%s/cgi-bin/config.php",
           LIDAR_IP.toString().c_str());
  char referer[64];
  snprintf(referer, sizeof(referer), "http://%s/config.html",
           LIDAR_IP.toString().c_str());

  bool all_ok = true;
  for (int i = 0; i < 2; i++) {
    HTTPClient http;
    http.setTimeout(5000);
    if (!http.begin(url)) {
      Serial.printf("[cfg] %s: could not open %s\n", what[i], url);
      all_ok = false;
      continue;
    }
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    http.addHeader("Referer", referer);      // required, see above

    int code = http.POST((uint8_t *)cmds[i], strlen(cmds[i]));
    if (code > 0) {
      Serial.printf("[cfg] %s: HTTP %d\n", what[i], code);
      if (code != HTTP_CODE_OK) all_ok = false;
    } else {
      Serial.printf("[cfg] %s: failed, %s\n", what[i],
                    http.errorToString(code).c_str());
      all_ok = false;
    }
    http.end();
    delay(300);      // let the CGI finish its EEPROM write before the next one
  }
  return all_ok;
}

static void on_eth_event(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("[eth] link up");
      link_up_ms = millis();
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
    case ARDUINO_EVENT_ETH_START:
      Serial.printf("[eth] ready, ip %s\n", ETH.localIP().toString().c_str());
      if (!eth_up) {
        udp.begin(SCAN_PORT);
        Serial.printf("[udp] listening on %u\n", SCAN_PORT);
        eth_up = true;
      }
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("[eth] link DOWN");
      eth_up = false;
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nLakiBeam -> ArduPilot proximity bridge");

  Serial1.begin(FC_BAUD, SERIAL_8N1, FC_RX, FC_TX);

  // Red from the first instruction, so a board that dies during init shows a
  // fault rather than nothing at all.
  neopixelWrite(LED_PIN, LED_LEVEL, 0, 0);

  sectors_reset();

  Network.onEvent(on_eth_event);

  SPI.begin(ETH_SCK, ETH_MISO, ETH_MOSI);

  eth_init_ok = ETH.begin(ETH_PHY_W5500, 1, ETH_CS, ETH_IRQ, ETH_RST, SPI);
  if (!eth_init_ok) {
    Serial.println("[eth] W5500 init FAILED - check wiring and power");
  }

  // Static addressing. The sensor sends to a fixed host address and there is
  // no DHCP server on this link, so DHCP would simply stall.
  if (!ETH.config(LOCAL_IP, GATEWAY, NETMASK)) {
    Serial.println("[eth] static IP config failed");
  }

  Serial.printf("[cfg] expecting sensor at %s, scan port %u\n",
                LIDAR_IP.toString().c_str(), SCAN_PORT);
}

void loop() {
  // Drain everything waiting. At ~215 packets/sec this keeps the socket clear.
  int n;
  while ((n = udp.parsePacket()) > 0) {
    int len = udp.read(pkt, sizeof(pkt));
    if (len >= PKT_LEN) {
      saw_return = false;
      decode_packet(pkt, len);
      packets_in++;
      packets_total++;
      last_packet_ms = millis();
      if (saw_return) last_return_ms = last_packet_ms;
    }
  }

  led_update();

  uint32_t now = millis();

  // Enable the laser and rotor, once, only if the sensor has never spoken.
  // Guarded on packets_total rather than run unconditionally at boot so a
  // sensor that is already scanning is left alone.
  if (eth_up && !config_done && packets_total == 0 &&
      now - link_up_ms >= CONFIG_AFTER_MS) {
    config_done = true;
    Serial.println("[cfg] Link is up but silent. Enabling laser and rotor.");
    if (sensor_configure()) {
      Serial.println("[cfg] Accepted. Scan data should start within a second.");
    } else {
      Serial.println("[cfg] At least one command failed. Proximity output will");
      Serial.println("      be empty, which ArduPilot cannot tell from clear air.");
    }
    last_send_ms = now;
  }

  if (now - last_send_ms >= SEND_INTERVAL_MS) {
    last_send_ms = now;

    /*
     * Health reporting, and it works by going quiet.
     *
     * Sending a frame every 100ms regardless of whether the sensor spoke means
     * a dead sensor produces a confident stream of "nothing anywhere". That is
     * indistinguishable from open ground, and it is the dangerous direction to
     * fail in: the autopilot believes it can see, and believes the way is
     * clear.
     *
     * Saying nothing is honest. ArduPilot's proximity backend times out within
     * a few hundred milliseconds of the last message and marks itself
     * unhealthy, which shows up in SYS_STATUS and which avoidance treats as a
     * fault rather than as clear air.
     *
     * Keyed on packets arriving, NOT on returns. A sensor scanning an empty
     * field legitimately reports nothing in range and is perfectly healthy.
     * The question is whether the LiDAR is talking, not whether it can see
     * anything.
     */
    bool sensor_alive = packets_total > 0 && (now - last_packet_ms) < SENSOR_STALE_MS;

    if (sensor_alive) {
      send_obstacle_distance();
      if (!was_alive) {
        Serial.println("[health] sensor back. Resuming OBSTACLE_DISTANCE.");
        was_alive = true;
      }
    } else {
      frames_muted++;
      if (was_alive) {
        Serial.printf("[health] no scan data for %dms. Going quiet so ArduPilot\n",
                      SENSOR_STALE_MS);
        Serial.println("         marks proximity unhealthy instead of believing");
        Serial.println("         the world is empty.");
        was_alive = false;
      }
    }

    // Clear after sending so each message reflects one window, not a smear of
    // everything since boot. A stale sector is worse than an unknown one.
    sectors_reset();
  }

  if (now - last_stat_ms >= 5000) {
    last_stat_ms = now;
    Serial.printf("[stat] packets in %lu, mavlink out %lu, muted %lu, link %s\n",
                  packets_in, frames_out, frames_muted, eth_up ? "up" : "down");
    if (packets_in == 0 && eth_up) {
      Serial.println("       No scan data, and the laser/rotor enable was");
      Serial.println("       already sent. Output is muted, so ArduPilot should");
      Serial.println("       be reporting proximity unhealthy. See ../README.md.");
    }
    packets_in = 0;
    frames_out = 0;
    frames_muted = 0;
  }
}
