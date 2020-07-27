#include <StepperDriver.h>
// https://github.com/DIMRobotics/ArduinoStepperDriver

/* Axis identifiers inside the library; on them then
   Can be controlled specific. You can do several axles,
   The maximum number in the native assembly is 3. It is treated by editing
   headerfile, there is a constant NUM_AXSIS
*/
axis_t bl, br, fl, fr;

float MaxSpeed = 3.14; // rad/s
#define PulseRev 3200

const float SECONDS_TO_MICROSECONDS = 1000000;
const float STEPPER_CONVERSION = SECONDS_TO_MICROSECONDS * 2 * PI / PulseRev;

#define bl_pulse 49
#define bl_dir 48
#define br_pulse 51
#define br_dir 50
#define fl_pulse 47
#define fl_dir 46
#define fr_pulse 45
#define fr_dir 44
#define PIN_ENABLE_ALL 40

void setup()
{

  Serial.begin(115200);
  StepperDriver.init();
  delay(100);
//  Serial.print("rad conversion: ");
//  Serial.println(RAD_PER_SEC_TO_STEPS_PER_SEC);
//  Serial.print("sec conversion: ");
//  Serial.println(SECONDS_TO_MICROSECONDS);

  bl = StepperDriver.newAxis(bl_pulse, bl_dir, 255, PulseRev);
  br = StepperDriver.newAxis(br_pulse, br_dir, 255, PulseRev);
  fr = StepperDriver.newAxis(fr_pulse, fr_dir, 255, PulseRev);
  fl = StepperDriver.newAxis(fl_pulse, fl_dir, 255, PulseRev);


  StepperDriver.enable(bl);
  StepperDriver.enable(br);
  StepperDriver.enable(fl);
  StepperDriver.enable(fr);
  digitalWrite(PIN_ENABLE_ALL, LOW);

  Serial.println("TCCR2A: ");
  Serial.println(TCCR2A, BIN);
  Serial.println("TCCR2B: ");
  Serial.println(TCCR2B, BIN);
  Serial.println("TIMSK2: ");
  Serial.println(TIMSK2, BIN);
  
}

void writeAll(float vel)
{
  Serial.print("Speed: ");
  Serial.println(vel, 4);
  Serial.print("Delays: [ ");
  writeSpeed(fl, vel);
  writeSpeed(fr, vel);
  writeSpeed(br, vel);
  writeSpeed(bl, vel);
  Serial.println("]");
}

void writeSpeed(axis_t motor, float vel)
{
  uint16_t delay_us = 0; // This works for if vel is 0
  
  // Compute motor direction
  if(vel > 0)
  {
      StepperDriver.setDir(motor, FORWARD);
  }
  else
  {
      vel = -vel;
      StepperDriver.setDir(motor, BACKWARD); 
  }
  
  // Compute delay between steps to achieve desired velocity
  if(vel != 0)
  {
      delay_us = static_cast<uint16_t>(STEPPER_CONVERSION / vel);
  }

  Serial.print(delay_us);
  Serial.print(", ");
  
  // Update motor
  StepperDriver.setDelay(motor, delay_us);
}

void loop()
{
   
   writeAll(MaxSpeed);  
   delay(2000);

   writeAll(0);

   delay(500);

   writeAll(-1*MaxSpeed);

   delay(2000);

   writeAll(0);

   delay(500);

}
