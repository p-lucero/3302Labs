#include <sparki.h>

void setup() {
  sparki.RGB(RGB_YELLOW);
  delay(5000);
  sparki.RGB(RGB_OFF);
}

void loop() {
  float pingval = sparki.ping_single();
  sparki.clearLCD();
  sparki.print("Ping returned "); sparki.print(pingval); sparki.print(" cm");
  sparki.updateLCD();
  delay(500);
}
