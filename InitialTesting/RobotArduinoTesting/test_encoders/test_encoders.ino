// Pins
#define PIN_ENCA_1 21
#define PIN_ENCA_2 20
#define PIN_ENCA_3 18
#define PIN_ENCA_4 19

#define PIN_ENCB_1 25
#define PIN_ENCB_2 24
#define PIN_ENCB_3 22
#define PIN_ENCB_4 23

#include <Encoder.h>

Encoder encoder(PIN_ENCA_3, PIN_ENCB_3);

long pos = -999;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Encoder Test:");
}

void loop() {
  // Check for new value
  long newPos;
  newPos = encoder.read();
  if(newPos != pos)
  {
    Serial.print("Pos = ");
    Serial.print(newPos);
    Serial.println();
    pos = newPos;
  }

  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available())
  {
    Serial.read();
    Serial.println("Reset position to zero");
    encoder.write(0);
  }


}
