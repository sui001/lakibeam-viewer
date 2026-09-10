/*
 * Which pin is the onboard RGB LED on?
 *
 * ESP32-S3 SuperMini boards are made by several vendors and the answer is not
 * the same on all of them. Documented as 48, seen working at 47. Rather than
 * trust either, drive both and look at the board.
 *
 *   RED   for 2s  ->  the LED is on GPIO 47
 *   BLUE  for 2s  ->  the LED is on GPIO 48
 *
 * Distinct colours so a half-second glance is enough, and so a board that
 * somehow answers on both is obvious rather than confusing.
 */

#define LED_A  47
#define LED_B  48

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n\nOnboard LED pin test");
  Serial.println("RED means GPIO 47. BLUE means GPIO 48.");
}

void loop() {
  Serial.println("[led] GPIO 47, red");
  neopixelWrite(LED_A, 60, 0, 0);
  delay(2000);
  neopixelWrite(LED_A, 0, 0, 0);
  delay(500);

  Serial.println("[led] GPIO 48, blue");
  neopixelWrite(LED_B, 0, 0, 60);
  delay(2000);
  neopixelWrite(LED_B, 0, 0, 0);
  delay(500);
}
