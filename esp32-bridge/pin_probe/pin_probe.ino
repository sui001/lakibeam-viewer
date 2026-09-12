/*
 * Which physical pad is which GPIO?
 *
 * Perf boards carry their own row numbers, modules carry theirs, and neither
 * is the ESP32's GPIO numbering. Reading a number off a silkscreen and
 * assuming it is a GPIO is an easy and expensive mistake.
 *
 * So stop reading silkscreen and ask the chip. Every free pin is held high
 * with an internal pull-up. Touch a jumper from GND to any pad and this prints
 * the GPIO number that went low.
 *
 * Pins 8 to 13 carry the W5500 and are driven by it, so they are watched but
 * reported separately: they may flicker on their own, and that flicker is
 * itself proof of what the W5500 is wired to.
 */

// Everything the SuperMini brings out to headers.
static const int FREE_PINS[] = {1, 3, 4, 5, 6, 7};
static const int W5500_PINS[] = {8, 9, 10, 11, 12, 13};

#define N_FREE  (sizeof(FREE_PINS) / sizeof(FREE_PINS[0]))
#define N_W5500 (sizeof(W5500_PINS) / sizeof(W5500_PINS[0]))

// GPIO 2 is deliberately absent. It is the strapping pin this board misbehaves
// on, and pulling it about during boot is not worth the one extra answer.

static bool was_low[64];

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n\nPin probe");
  Serial.println("Touch a GND jumper to any pad. The GPIO behind it prints here.");
  Serial.print("Watching free pins:");
  for (unsigned i = 0; i < N_FREE; i++) {
    pinMode(FREE_PINS[i], INPUT_PULLUP);
    Serial.printf(" %d", FREE_PINS[i]);
  }
  Serial.println();
  Serial.print("Watching W5500 pins:");
  for (unsigned i = 0; i < N_W5500; i++) {
    pinMode(W5500_PINS[i], INPUT_PULLUP);
    Serial.printf(" %d", W5500_PINS[i]);
  }
  Serial.println();
  Serial.println("W5500 pins may flicker unprompted. That is the module "
                 "driving them,");
  Serial.println("which is itself the answer to what it is wired to.");
}

static void watch(const int *pins, unsigned n, const char *tag) {
  for (unsigned i = 0; i < n; i++) {
    int p = pins[i];
    bool low = (digitalRead(p) == LOW);
    if (low && !was_low[p]) Serial.printf("  GPIO %-2d  LOW   (%s)\n", p, tag);
    if (!low && was_low[p]) Serial.printf("  GPIO %-2d  released\n", p);
    was_low[p] = low;
  }
}

void loop() {
  watch(FREE_PINS, N_FREE, "free");
  watch(W5500_PINS, N_W5500, "W5500");
  delay(30);      // crude debounce, a probed pad is never that fast
}
