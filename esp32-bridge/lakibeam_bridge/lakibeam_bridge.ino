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
#include <common/mavlink.h>

// ── Wiring ───────────────────────────────────────────────────────────────────
// W5500 on SPI. These avoid the SuperMini's strapping pins (0, 2, 45, 46) and
// the onboard RGB LED (47). Check them against your board before flashing.
#define ETH_SCK    12
#define ETH_MISO   13
#define ETH_MOSI   11
#define ETH_CS     10
#define ETH_IRQ    14
#define ETH_RST     9

// Serial link to the flight controller.
#define FC_TX      17
#define FC_RX      18
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
#define MOUNT_YAW_DEG 0.0f

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

  float a = angle_deg + MOUNT_YAW_DEG;
  a = fmodf(a, 360.0f);
  if (a < 0) a += 360.0f;

  int idx = (int)(a / SECTOR_DEG);
  if (idx < 0 || idx >= NUM_SECTORS) return;

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

static void on_eth_event(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("[eth] link up");
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

  sectors_reset();

  Network.onEvent(on_eth_event);

  SPI.begin(ETH_SCK, ETH_MISO, ETH_MOSI);

  if (!ETH.begin(ETH_PHY_W5500, 1, ETH_CS, ETH_IRQ, ETH_RST, SPI)) {
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
      decode_packet(pkt, len);
      packets_in++;
    }
  }

  uint32_t now = millis();

  if (now - last_send_ms >= SEND_INTERVAL_MS) {
    last_send_ms = now;
    send_obstacle_distance();
    // Clear after sending so each message reflects one window, not a smear of
    // everything since boot. A stale sector is worse than an unknown one.
    sectors_reset();
  }

  if (now - last_stat_ms >= 5000) {
    last_stat_ms = now;
    Serial.printf("[stat] packets in %lu, mavlink out %lu, link %s\n",
                  packets_in, frames_out, eth_up ? "up" : "down");
    if (packets_in == 0 && eth_up) {
      Serial.println("       No scan data. The sensor's laser and rotor are");
      Serial.println("       both off by default and the setting persists to");
      Serial.println("       EEPROM. See ../README.md.");
    }
    packets_in = 0;
    frames_out = 0;
  }
}
