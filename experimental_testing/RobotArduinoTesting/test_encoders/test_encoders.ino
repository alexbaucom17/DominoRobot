// Pins
#define PIN_ENCA_1 21
#define PIN_ENCA_2 20
#define PIN_ENCA_3 19
#define PIN_ENCA_4 18

#define PIN_ENCB_1 25
#define PIN_ENCB_2 23
#define PIN_ENCB_3 24
#define PIN_ENCB_4 22

#include <Encoder.h>

Encoder e[4] = { Encoder(PIN_ENCA_1, PIN_ENCB_1),
                 Encoder(PIN_ENCA_2, PIN_ENCB_2), 
                 Encoder(PIN_ENCA_3, PIN_ENCB_3), 
                 Encoder(PIN_ENCA_4, PIN_ENCB_4) } ;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Encoder Test:");
}

void loop() {
  
  Serial.print("Pos: [");
  for (int i = 0; i < 4; ++i )
  {
    Serial.print(e[i].read());
    Serial.print(", ");
  }
  Serial.println("]");


  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available())
  {
    Serial.read();
    Serial.println("Reset position to zero");
    e[0].write(0);
    e[1].write(0);
    e[2].write(0);
    e[3].write(0);
  }

  delay(100);


}
