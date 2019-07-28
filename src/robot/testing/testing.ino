// Test out a some basic motor control

#define ENABLE1_PIN 51
#define DIR1_PIN 52
#define PWM1_PIN 7

bool enable = HIGH;
bool dir = HIGH;
int stage = 1;
int power = 255;

void setup() {
  // put your setup code here, to run once:
  pinMode(ENABLE1_PIN,OUTPUT);
  pinMode(DIR1_PIN,OUTPUT);
  pinMode(PWM1_PIN,OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (stage == 1)
  {
    enable = HIGH;
    dir = HIGH;
    power = 255;
    stage = 2;
    Serial.println("Stage 1");
  }
  else if (stage == 2)
  {
    enable = LOW;
    dir = HIGH;
    power = 0;
    stage = 3;
    Serial.println("Stage 3");
  }
  else if (stage == 3)
  {
    enable = HIGH;
    dir = LOW;
    power = 128;
    stage = 4;
    Serial.println("Stage 2");
  }
  else if (stage == 4)
  {
    enable = LOW;
    dir = HIGH;
    power = 0;
    stage = 1;
    Serial.println("Stage 3");
  }

  digitalWrite(ENABLE1_PIN, enable);
  digitalWrite(DIR1_PIN, dir);
  analogWrite(PWM1_PIN, power);
  delay(2000);
  

}
