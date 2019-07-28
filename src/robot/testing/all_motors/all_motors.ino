// Pinouts
#define PIN_ENABLE 52
#define PIN_FR_PWM 3
#define PIN_FR_DIR 49
#define PIN_FL_PWM 2
#define PIN_FL_DIR 51
#define PIN_BR_PWM 5
#define PIN_BR_DIR 50
#define PIN_BL_PWM 6
#define PIN_BL_DIR 48

// Motor index
#define FR 0
#define FL 1
#define BR 2
#define BL 3

// Motor arrays
const int PWM_ARRAY[4] = {PIN_FR_PWM, PIN_FL_PWM, PIN_BR_PWM, PIN_BL_PWM};
const int DIR_ARRAY[4] = {PIN_FR_DIR, PIN_FL_DIR, PIN_BR_DIR, PIN_BL_DIR};
const int SPEED_ARRAY[4] = {130, 75, 75, 100}; 

const int NUM_DIRECTIONS = 4;

// Direction vectors
const int MOTOR_DIRECTIONS[NUM_DIRECTIONS][4] = 
{
  {1, 1, 1, 1},   //Forward
  {0, 0, 0, 0},   //Backward
  {1, 0, 0, 1},   //Left
  {0, 1, 1, 0}   //Right
//  {1, 0, 1, 0},   //Forward-Left
//  {1, 0, 0, 1},   //Forward-Right
//  {0, 1, 0, 1},   //Backward-Left
//  {0, 1, 1, 0}    //Backward-Right
};


void commandMotors(int direction)
{
  for(int i = 0; i < 4; i++)
  {
    analogWrite(PWM_ARRAY[i],SPEED_ARRAY[i]);
    digitalWrite(DIR_ARRAY[i],MOTOR_DIRECTIONS[direction][i]);
  } 
  digitalWrite(PIN_ENABLE,HIGH);
}


void setup() 
{
  // Setup pin modes
  pinMode(PIN_ENABLE,OUTPUT);
  for(int i = 0; i < 4; i++)
  {
    pinMode(PWM_ARRAY[i],OUTPUT);
    pinMode(DIR_ARRAY[i],OUTPUT);
  }
  
  Serial.begin(9600);
}


int stage = 0;
void loop() 
{
  // Command motors on
  commandMotors(stage);
  Serial.print("On: ");
  Serial.println(stage);

  // Wait for a little bit
  delay(1000);

  // Turn motors off and let coast
  digitalWrite(PIN_ENABLE,HIGH);
  for(int i = 0; i < 4; i++)
  {
    analogWrite(PWM_ARRAY[i],0);
  } 
  Serial.println("Off");

  // Wait for a little bit
  delay(1500);

  // Increment stage
  if(stage == NUM_DIRECTIONS-1)
  {
    stage = 0;
  }
  else
  {
    stage++;
  }
}
