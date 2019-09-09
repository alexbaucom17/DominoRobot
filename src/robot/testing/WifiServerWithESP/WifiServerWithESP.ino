void setup() {
  // put your setup code here, to run once:

  // Communication with the host computer
  Serial.begin(9600); 
  Serial.println("Wifi client starting"); 

  // Start the serial for communication with the ESP8266
  Serial3.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:

   // Forward data from wifi to serial terminal
   if ( Serial3.available() )   {  Serial.write( Serial3.read() );  }


   // Forward data from serial terminal to wifi
   if ( Serial.available() )       {  Serial3.write( Serial.read() );  }

}
