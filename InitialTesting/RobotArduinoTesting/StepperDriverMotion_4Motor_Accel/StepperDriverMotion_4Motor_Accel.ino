#include <StepperDriver.h>
// https://github.com/DIMRobotics/ArduinoStepperDriver

/* Axis identifiers inside the library; on them then
   Can be controlled specific. You can do several axles,
   The maximum number in the native assembly is 3. It is treated by editing
   headerfile, there is a constant NUM_AXSIS
*/
axis_t bl, br, fl, fr;

int MaxSpeed = 50;
int acc_factor = 1;
int pause_time = 1000;
#define PulseRev 1600
#define wheelSize = 100 //in mm

#define bl_pulse 49
#define bl_dir 48
#define br_pulse 51
#define br_dir 50
#define fl_pulse 47
#define fl_dir 46
#define fr_pulse 45
#define fr_dir 44

void setup()
{

  Serial.begin(115200);
  StepperDriver.init();

  bl = StepperDriver.newAxis(bl_pulse, bl_dir, 255, PulseRev);
  br = StepperDriver.newAxis(br_pulse, br_dir, 255, PulseRev);
  fr = StepperDriver.newAxis(fr_pulse, fr_dir, 255, PulseRev);
  fl = StepperDriver.newAxis(fl_pulse, fl_dir, 255, PulseRev);


  StepperDriver.enable(bl);
  StepperDriver.enable(br);
  StepperDriver.enable(fl);
  StepperDriver.enable(fr);
}

void writeAll(int speed)
{
  StepperDriver.write(fl, speed);
  StepperDriver.write(fr, speed);
  StepperDriver.write(br, speed);
  StepperDriver.write(bl, speed);
}

void loop()
{
   
   writeAll(MaxSpeed);  
   delay(1000);

   writeAll(0);

   delay(500);

   writeAll(-1*MaxSpeed);

   delay(1000);

   writeAll(0);

   delay(500);

//  for (int x = 1; x < 5; x++) {
//    int cur_acc = acc_factor * x;
//    Serial.print("Acc: ");
//    Serial.println(cur_acc);
//    Serial.println("Accelerating forwards");
//    accelLinear(1, MaxSpeed, cur_acc);
//    delay(pause_time);
//    Serial.println("Decelerating forwards");
//    decelLinear(1, MaxSpeed, cur_acc);
//    delay(pause_time);
//    Serial.println("Accelerating backwards");
//    accelLinear(0, MaxSpeed, cur_acc);
//    delay(pause_time);
//    Serial.println("Decelerating backwards");
//    decelLinear(0, MaxSpeed, cur_acc);
//  }
//
//  delay(2000);
//
//  rotateAngle(300);
//  delay(2000);
//  rotateAngle(-300);
//  delay(2000);

}

//
//void accelLinear(int dir, int topspeed, int accel) {
//  int mspeed = 0;
//  if (dir == 0) {
//    StepperDriver.setDir(br, FORWARD);
//    StepperDriver.setDir(bl, BACKWARD);
//    StepperDriver.setDir(fr, FORWARD);
//    StepperDriver.setDir(fl, BACKWARD);
//  }
//  else {
//    StepperDriver.setDir(br, BACKWARD);
//    StepperDriver.setDir(bl, FORWARD);
//    StepperDriver.setDir(fr, BACKWARD);
//    StepperDriver.setDir(fl, FORWARD);
//  }
//  while (mspeed < topspeed) {
//    mspeed += accel;
//    Serial.print("Speed: ");
//    Serial.println(mspeed);
//    StepperDriver.setSpeed(fl, mspeed);
//    StepperDriver.setSpeed(fr, mspeed);
//    StepperDriver.setSpeed(bl, mspeed);
//    StepperDriver.setSpeed(br, mspeed);
//    //delayMicroseconds(accel);
//  }
//
//}
//
//void decelLinear(int dir, int currspeed, int decel) {
//  int mspeed = currspeed;
//  if (dir == 0) {
//    StepperDriver.setDir(br, FORWARD);
//    StepperDriver.setDir(bl, BACKWARD);
//    StepperDriver.setDir(fr, FORWARD);
//    StepperDriver.setDir(fl, BACKWARD);
//  }
//  else {
//    StepperDriver.setDir(br, BACKWARD);
//    StepperDriver.setDir(bl, FORWARD);
//    StepperDriver.setDir(fr, BACKWARD);
//    StepperDriver.setDir(fl, FORWARD);
//  }
//  while (mspeed > 0) {
//    mspeed -= decel;
//    if (mspeed < 0) {
//      mspeed = 0;
//      StepperDriver.stop(fl);
//      StepperDriver.stop(fr);
//      StepperDriver.stop(bl);
//      StepperDriver.stop(br);
//    }
//    Serial.print("Speed: ");
//    Serial.println(mspeed);
//    StepperDriver.setSpeed(fl, mspeed);
//    StepperDriver.setSpeed(fr, mspeed);
//    StepperDriver.setSpeed(bl, mspeed);
//    StepperDriver.setSpeed(br, mspeed);
//    //delayMicroseconds(accel);
//  }
//}
//
//void rotateAngle(int angle) {
//  int stepspeed = 240;
//  if(angle<0){
//    stepspeed *= -1;
//    angle *= -1;
//  }
//  Serial.println(angle);
//  float divangle = float(angle)/360;
//  
//  Serial.println(divangle);
//  int adjust = int(divangle * 100);
//  Serial.println(adjust);
//  int steps = (adjust * PulseRev)/100;
//  Serial.println(steps);
//  StepperDriver.write(fl,stepspeed, steps);
//  StepperDriver.write(fr, stepspeed, steps);
//  StepperDriver.write(bl, stepspeed, steps);
//  StepperDriver.write(br, stepspeed, steps);
//}
