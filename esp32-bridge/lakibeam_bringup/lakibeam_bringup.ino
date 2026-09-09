/*
 * LakiBeam bridge bring-up
 *
 * Proves the hardware half of the bridge before any of the MAVLink half
 * exists: SPI to the W5500, Ethernet link, UDP from the sensor, and the packet
 * decode. Everything is reported over USB serial. No flight controller, no
 * MAVLink library, nothing connected to the Cube.
 *
 * Flash this first. If it reports packets and sensible distances, the only
 * unproven thing left in lakibeam_bridge.ino is the MAVLink output.
 *
 * Hardware: ESP32-S3 SuperMini + W5500 Lite. See ../README.md for wiring.
 */

#include <ETH.h>
#include <SPI.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>

// ── Wiring ───────────────────────────────────────────────────────────────────
// All within GPIO 1-13, which is what the SuperMini breaks out to headers.
// 10-13 are the S3's native FSPI block. GPIO 2 avoided as a strapping pin.
#define ETH_SCK    12
#define ETH_MISO   13
#define ETH_MOSI   11
#define ETH_CS     10
#define ETH_IRQ     8      // -1 if your module does not break INT out
#define ETH_RST     9      // -1 if your module does not break RST out

// ── Network ──────────────────────────────────────────────────────────────────
static const IPAddress LOCAL_IP (192, 168, 198, 1);
static const IPAddress NETMASK  (255, 255, 255, 0);
static const IPAddress GATEWAY  (192, 168, 198, 1);
static const IPAddress LIDAR_IP (192, 168, 198, 2);
static const uint16_t  SCAN_PORT = 2368;

// How long to wait, after the link comes up, before deciding the sensor is
// silent because its laser and rotor were left off rather than because it is
// still starting.
#define CONFIG_AFTER_MS 5000

// ── Packet format ────────────────────────────────────────────────────────────
#define PKT_LEN       1206
#define BLOCKS        12
#define BLOCK_POINTS  16
#define BLOCK_SIZE    100
#define AZ_INVALID    0xFFFF

#define REPORT_MS     2000

static WiFiUDP  udp;
static uint8_t  pkt[PKT_LEN + 64];
static bool     eth_up = false;
static bool     udp_open = false;
static bool     eth_init_ok = false;   // did the W5500 answer over SPI at all
static bool     config_done = false;   // laser/rotor enable already attempted
static uint32_t link_up_ms = 0;
static uint32_t packets_total = 0;     // survives the reporting window reset

// Stats for the current reporting window.
static uint32_t packets = 0, runts = 0, points_valid = 0, points_zero = 0;
static uint16_t dist_min = 0xFFFF, dist_max = 0;
static float    az_min = 999.0f, az_max = -999.0f;
static IPAddress last_sender;
static uint32_t last_report_ms = 0;

static void stats_reset() {
  packets = runts = points_valid = points_zero = 0;
  dist_min = 0xFFFF;
  dist_max = 0;
  az_min = 999.0f;
  az_max = -999.0f;
}

/*
 * Decode one packet for statistics only. Same interpolation as the bridge:
 * azimuth arrives once per sub-packet, so the 16 points inside it are spread
 * between this azimuth and the next. If that is wrong the angle span below
 * comes out wrong too, which is the point of printing it.
 */
static void decode_stats(const uint8_t *p) {
  for (int b = 0; b < BLOCKS - 1; b++) {
    const uint8_t *blk = p + b * BLOCK_SIZE;

    uint16_t az = (uint16_t)(blk[2] | (blk[3] << 8));
    if (az == AZ_INVALID) continue;

    uint16_t az_next = (uint16_t)(p[(b + 1) * BLOCK_SIZE + 2] |
                                  (p[(b + 1) * BLOCK_SIZE + 3] << 8));
    if (az_next == AZ_INVALID) az_next = az;

    az      %= 36000;
    az_next %= 36000;

    int32_t span = (int32_t)az_next - (int32_t)az;
    if (span < 0) span += 36000;
    float step = (float)span / BLOCK_POINTS;

    for (int j = 0; j < BLOCK_POINTS; j++) {
      const uint8_t *pt = blk + 4 + j * 6;
      uint16_t dist_mm = (uint16_t)(pt[0] | (pt[1] << 8));
      float a = fmodf((az + j * step) * 0.01f, 360.0f);

      if (dist_mm == 0) { points_zero++; continue; }

      points_valid++;
      if (dist_mm < dist_min) dist_min = dist_mm;
      if (dist_mm > dist_max) dist_max = dist_mm;
      if (a < az_min) az_min = a;
      if (a > az_max) az_max = a;
    }
  }
}

static void report() {
  Serial.println();
  // Repeated every window on purpose. Printing this once at boot means it has
  // scrolled away by the time anyone opens a serial monitor, which is exactly
  // when it is wanted.
  Serial.printf("w5500     %s\n", eth_init_ok ? "responding over SPI"
                                              : "NOT RESPONDING over SPI");
  Serial.printf("link      %s\n", eth_up ? "UP" : "DOWN");
  Serial.printf("local ip  %s\n", ETH.localIP().toString().c_str());

  if (!eth_init_ok) {
    Serial.println("  The W5500 never answered, so nothing downstream can work.");
    Serial.println("  This is SPI wiring or power, not the sensor or the cable.");
    Serial.println("  Check 3V3 and GND first, then that MISO and MOSI are not");
    Serial.println("  swapped, then CS.");
    return;
  }

  if (!eth_up) {
    Serial.println("  W5500 is alive but sees no Ethernet link partner.");
    Serial.println("  Check the cable is seated at both ends and that the sensor");
    Serial.println("  has 12V on its barrel jack. The sensor's PHY only comes up");
    Serial.println("  when powered, so a dark sensor reads the same as a dead");
    Serial.println("  cable. The W5500 Lite's own Link LED is the ground truth.");
    return;
  }

  Serial.printf("packets   %lu in %d ms (%lu runts)\n", packets, REPORT_MS, runts);

  if (packets == 0) {
    Serial.println("  Link is up but no scan data.");
    Serial.println("  Most likely the sensor's laser and rotor are off. They are");
    Serial.println("  both off by default, enabling the laser alone leaves rpm at");
    Serial.println("  0, and the setting persists to EEPROM. Run the Python viewer");
    Serial.println("  once from a PC on this network, or POST -en 1 and -freq 5.");
    return;
  }

  Serial.printf("from      %s\n", last_sender.toString().c_str());
  Serial.printf("points    %lu returns, %lu empty\n", points_valid, points_zero);

  if (points_valid > 0) {
    Serial.printf("distance  %u to %u mm\n", dist_min, dist_max);
    Serial.printf("azimuth   %.1f to %.1f deg\n", az_min, az_max);
    Serial.println("  Expect roughly 430 packets per window, distances inside");
    Serial.println("  about 20 to 16000 mm, and azimuth inside 45 to 315 deg.");
  } else {
    Serial.println("  Packets arriving but every return is zero. The rotor may be");
    Serial.println("  spinning with the laser off. Check -en 1 was accepted.");
  }
}

/*
 * Turn the laser and the rotor on.
 *
 * They are separate, both off by default, and the setting persists to the
 * sensor's EEPROM. Enabling the laser alone leaves rpm at 0 and produces no
 * points at all, which looks exactly like dead hardware. A unit that arrives
 * "broken" has usually just been switched off in software by whoever had it.
 *
 * Two things make this fussy. The CGI needs a Referer header or the request
 * hangs until timeout instead of returning an error. And the sensor's control
 * plane starves once it is scanning, so this only runs while it is still
 * quiet, which is the only time it is reliable anyway.
 */
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
      eth_up = true;
      link_up_ms = millis();
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
    case ARDUINO_EVENT_ETH_START:
      Serial.printf("[eth] ready, ip %s\n", ETH.localIP().toString().c_str());
      if (!udp_open) {
        udp.begin(SCAN_PORT);
        udp_open = true;
        Serial.printf("[udp] listening on %u\n", SCAN_PORT);
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
  delay(2000);            // let USB CDC enumerate so the banner is not lost
  Serial.println("\n\nLakiBeam bridge bring-up");
  Serial.printf("SPI  sck %d  miso %d  mosi %d  cs %d  irq %d  rst %d\n",
                ETH_SCK, ETH_MISO, ETH_MOSI, ETH_CS, ETH_IRQ, ETH_RST);

  stats_reset();
  Network.onEvent(on_eth_event);

  SPI.begin(ETH_SCK, ETH_MISO, ETH_MOSI);

  eth_init_ok = ETH.begin(ETH_PHY_W5500, 1, ETH_CS, ETH_IRQ, ETH_RST, SPI);
  if (!eth_init_ok) {
    Serial.println("[eth] W5500 init FAILED");
    Serial.println("      The chip did not answer over SPI at all, so this is");
    Serial.println("      wiring or power, not the sensor. Check 3V3 and GND");
    Serial.println("      first, then MISO and MOSI are not swapped.");
  } else {
    Serial.println("[eth] W5500 init ok");
  }

  // Static addressing. The sensor sends to a fixed host address and there is
  // no DHCP server on this link, so DHCP would simply stall.
  if (!ETH.config(LOCAL_IP, GATEWAY, NETMASK)) {
    Serial.println("[eth] static IP config failed");
  }

  Serial.println("Reporting every 2 seconds.");
}

void loop() {
  int n;
  while ((n = udp.parsePacket()) > 0) {
    int len = udp.read(pkt, sizeof(pkt));
    if (len >= PKT_LEN) {
      last_sender = udp.remoteIP();
      decode_stats(pkt);
      packets++;
      packets_total++;
    } else {
      runts++;
    }
  }

  uint32_t now = millis();

  // Enable the laser and rotor, once, only if the sensor is silent. Guarded on
  // packets_total rather than run unconditionally at boot: the settings persist
  // to EEPROM, so a sensor that is already scanning needs no write, and there
  // is no reason to spend EEPROM cycles on every power cycle.
  if (eth_up && !config_done && packets_total == 0 &&
      now - link_up_ms >= CONFIG_AFTER_MS) {
    config_done = true;
    Serial.println("\n[cfg] Link is up but silent. Enabling laser and rotor.");
    if (sensor_configure()) {
      Serial.println("[cfg] Accepted. Scan data should start within a second.");
    } else {
      Serial.println("[cfg] At least one command failed. If the sensor stays");
      Serial.println("      quiet, check it answers at 192.168.198.2 at all.");
    }
    last_report_ms = now;    // do not fire a stale report straight after
  }

  if (now - last_report_ms >= REPORT_MS) {
    last_report_ms = now;
    report();
    stats_reset();
  }
}
