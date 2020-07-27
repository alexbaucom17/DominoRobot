
#define LIFTER_PIN_0 1
#define LIFTER_PIN_1 2
#define LIFTER_PIN_2 3
#define FEEDBACK_PIN 4


void setup ()
{
    Serial.begin(115200);
    pinMode(LIFTER_PIN_0, OUTPUT);
    pinMode(LIFTER_PIN_1, OUTPUT);
    pinMode(LIFTER_PIN_2, OUTPUT);
    pinMode(FEEDBACK_PIN, INPUT_PULLUP);
}

int getCommand()
{
    String instr = "";
    int outCmd = 0;
    if(Serial.available())
    {
        instr = Serial.readString();
        outCmd = instr.toInt();       
    }
    return outCmd;
}

void sendPos(int num)
{
    int p2 = num / 4;
    int r = num % 4;
    int p1 = r / 2;
    r = r % 2;
    int p0 = r / 1;

    digitalWrite(LIFTER_PIN_0, p0);
    digitalWrite(LIFTER_PIN_1, p1);
    digitalWrite(LIFTER_PIN_2, p2);
}

bool getFeedback()
{
    return !digitalRead(FEEDBACK_PIN);
}


void loop()
{

    int cmd = getCommand();

    bool in_progress = getFeedback();

    if(!in_progress && cmd > 0)
    {
        sendPos(cmd);
    }

    delay(100);

}
