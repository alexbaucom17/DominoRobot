#define PIN_ENABLE 52
#define PIN_BL_PWM 6
#define PIN_BL_DIR 48
#define FW 1
#define BW 0
#define SPEED_SLOW 50
#define SPEED_FAST 255


void setup() {
  // put your setup code here, to run once:
    pinMode(PIN_ENABLE,OUTPUT);
    pinMode(PIN_BL_PWM,OUTPUT);
    pinMode(PIN_BL_DIR,OUTPUT);
    Serial.begin(9600);
}


void loop() {
  // put your main code here, to run repeatedly:

  Serial.print("Forward");
  analogWrite(PIN_BL_PWM, SPEED_SLOW);
  digitalWrite(PIN_BL_DIR, FW);
  digitalWrite(PIN_ENABLE, HIGH);

  // Wait for a little bit
  delay(2000);

  // Stop
  digitalWrite(PIN_ENABLE, LOW);
  delay(500);
  
  Serial.print("Reverse");
  analogWrite(PIN_BL_PWM, SPEED_FAST);
  digitalWrite(PIN_BL_DIR, BW);
  digitalWrite(PIN_ENABLE, HIGH);

  // Wait for a little bit
  delay(2000);

  // Stop
  digitalWrite(PIN_ENABLE, LOW);
  delay(500);

}
