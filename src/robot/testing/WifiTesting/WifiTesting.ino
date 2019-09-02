

//  Arduino pin 15 (RX3) to ESP8266 TX
//  Arduino pin 14 (TX3) to voltage divider then to ESP8266 RX
//  Connect GND from the Arduiono to GND on the ESP8266
//  Pull ESP8266 CH_PD HIGH

// When a command is entered in to the serial monitor on the computer 
// the Arduino will relay it to the ESP8266
 
void setup() 
{
 
    Serial.begin(9600);     // communication with the host computer
 
    // Start the serial for communication with the ESP8266
    Serial3.begin(115200);  
 
    Serial.println("");
    Serial.println("Remember to to set Both NL & CR in the serial monitor.");
    Serial.println("Ready");
    Serial.println("");    
}
 
void loop() 
{
    // listen for communication from the ESP8266 and then write it to the serial monitor
    if ( Serial3.available() )   {  Serial.write( Serial3.read() );  }
 
    // listen for user input and send it to the ESP8266
    if ( Serial.available() )       {  Serial3.write( Serial.read() );  }
}
