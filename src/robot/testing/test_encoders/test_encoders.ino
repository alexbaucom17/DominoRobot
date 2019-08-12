// Pins
#define PIN_FR_ENC_A 20
#define PIN_FR_ENC_B 35
#define PIN_FL_ENC_A 21
#define PIN_FL_ENC_B 41
#define PIN_BR_ENC_A 18
#define PIN_BR_ENC_B 23
#define PIN_BL_ENC_A 19
#define PIN_BL_ENC_B 29

#include <Encoder.h>

Encoder encoderFR(PIN_FR_ENC_A, PIN_FR_ENC_B);
Encoder encoderFL(PIN_FL_ENC_A, PIN_FL_ENC_B);
Encoder encoderBR(PIN_BR_ENC_A, PIN_BR_ENC_B);
Encoder encoderBL(PIN_BL_ENC_A, PIN_BL_ENC_B);

long pos = -999;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Encoder Test:");
}

void loop() {
  // Check for new value
  long newPos;
  newPos = encoderFL.read();
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
    encoderFL.write(0);
  }


}
