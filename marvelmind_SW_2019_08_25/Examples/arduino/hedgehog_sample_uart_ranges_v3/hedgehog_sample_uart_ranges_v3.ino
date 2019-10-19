/*
 *   This example outputs to display distances from mobile beacon (hedgehog) to four stationary beacons
 */

/*
  The circuit:
 * HEDGEHOG serial data to digital pin 0 (RXD)
 *HC-SR04 Distance Sensor Trig pin to digital pin 2
 *HC-SR04 Distance Sensor Echo pin to digital pin 3
 * LCD RS pin to digital pin 8
 * LCD Enable pin to digital pin 9
 * LCD D4 pin to digital pin 4
 * LCD D5 pin to digital pin 5
 * LCD D6 pin to digital pin 6
 * LCD D7 pin to digital pin 7
 * LCD BL pin to digital pin 10
 *Vcc pin to  +5
 */

#include <stdlib.h>
#include <LiquidCrystal.h>

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//  MARVELMIND HEDGEHOG RELATED PART

#define HEDGEHOG_POS_PACKET_ID 0x0001
#define HEDGEHOG_POS_HIGHRES_PACKET_ID 0x0011
#define BEACONS_POS_PACKET_ID 0x0002
#define BEACONS_POS_HIGHRES_PACKET_ID 0x0012

#define HEDGEHOG_CM_DATA_SIZE 0x10
#define HEDGEHOG_MM_DATA_SIZE 0x16

long hedgehog_x, hedgehog_y;// coordinates of hedgehog (X,Y), mm
long hedgehog_z;// height of hedgehog, mm
int hedgehog_pos_updated;// flag of new data from hedgehog received
bool beacons_pos_appeared= false;

typedef struct {
 int adr;
 long x;// coordinates of beacon, mm
 long y;
 long z;
 int ok;
} tbeacon_pos;

#define MAX_BEACONS_SAVE 5
tbeacon_pos beacon_pos[MAX_BEACONS_SAVE];// beacons positions

///

//#define DISTANCE_SENSOR_ENABLED

#define HEDGEHOG_BUF_SIZE 128 
byte hedgehog_serial_buf[HEDGEHOG_BUF_SIZE];
byte hedgehog_serial_buf_ofs;
byte hedgehog_packet_size;

unsigned int hedgehog_data_id;
bool high_resolution_mode;

typedef union {byte b[2]; unsigned int w;int wi;} uni_8x2_16;
typedef union {byte b[4];float f;unsigned long v32;long vi32;} uni_8x4_32;

//    Marvelmind hedgehog support initialize
void setup_hedgehog() 
{int i;
  Serial.begin(500000); // hedgehog transmits data on 500 kbps  

  hedgehog_serial_buf_ofs= 0;
  hedgehog_pos_updated= 0;

  hedgehog_data_id= 0;

  for(i=0;i<MAX_BEACONS_SAVE;i++)
    {
      beacon_pos[i].ok= 0;
    }
}


float sqr(float x)
{
  return x*x;
}

long hedgehog_range(int beacon_index)
{float dx,dy,dz;  
  dx= ((float) (hedgehog_x - beacon_pos[beacon_index].x))/1000.0f;
  dy= ((float) (hedgehog_y - beacon_pos[beacon_index].y))/1000.0f;
  dz= ((float) (hedgehog_z - beacon_pos[beacon_index].z))/1000.0f;
  
  return sqrt(sqr(dx)+ sqr(dy)+ sqr(dz))*1000.0f;
}


// Marvelmind hedgehog service loop
void loop_hedgehog()
{int incoming_byte;
 int total_received_in_loop;
 int packet_received;
 bool good_byte;
 int i,n,ofs;
 uni_8x2_16 un16;
 uni_8x4_32 un32;

  total_received_in_loop= 0;
  packet_received= 0;
  while(Serial.available() > 0)
    {
      if (hedgehog_serial_buf_ofs>=HEDGEHOG_BUF_SIZE) 
      {
        hedgehog_serial_buf_ofs= 0;// restart bufer fill
        break;// buffer overflow
      }
      total_received_in_loop++;
      if (total_received_in_loop>200) break;// too much data without required header
      
      incoming_byte= Serial.read();
      good_byte= false;
      switch(hedgehog_serial_buf_ofs)
      {
        case 0:
        {
          good_byte= (incoming_byte = 0xff);
          break;
        }
        case 1:
        {
          good_byte= (incoming_byte = 0x47);
          break;
        }
        case 2:
        {
          good_byte= true;
          break;
        }
        case 3:
        {
          hedgehog_data_id= (((unsigned int) incoming_byte)<<8) + hedgehog_serial_buf[2];
          good_byte=   (hedgehog_data_id == HEDGEHOG_POS_PACKET_ID) ||
                       (hedgehog_data_id == HEDGEHOG_POS_HIGHRES_PACKET_ID) ||
                       (hedgehog_data_id == BEACONS_POS_PACKET_ID) ||
                       (hedgehog_data_id == BEACONS_POS_HIGHRES_PACKET_ID);
          break;
        }
        case 4:
        {
          // save required packet size   
          hedgehog_packet_size= incoming_byte + 7;
          switch(hedgehog_data_id)
          {
            case HEDGEHOG_POS_PACKET_ID:
            {
              good_byte= (incoming_byte == HEDGEHOG_CM_DATA_SIZE);
              break;
            }
            case HEDGEHOG_POS_HIGHRES_PACKET_ID:
            {
              good_byte= (incoming_byte == HEDGEHOG_MM_DATA_SIZE);
              break;
            }
            case BEACONS_POS_PACKET_ID:
            case BEACONS_POS_HIGHRES_PACKET_ID:
            {
              good_byte= true;
              break;
            }
          }
          break;
        }
        default:
        {
          good_byte= true;
          break;
        }
      }
      
      if (!good_byte)
        {
          hedgehog_serial_buf_ofs= 0;// restart bufer fill    
          hedgehog_data_id= 0;       
          continue;
        }     
        
      hedgehog_serial_buf[hedgehog_serial_buf_ofs++]= incoming_byte;
      if (hedgehog_serial_buf_ofs>5)
        { 
          if (hedgehog_serial_buf_ofs == hedgehog_packet_size)
            {// received packet with required header
              packet_received= 1;
              hedgehog_serial_buf_ofs= 0;// restart bufer fill
              break; 
            }
        }
    }
    

  if (packet_received)  
    {
      hedgehog_set_crc16(&hedgehog_serial_buf[0], hedgehog_packet_size);// calculate CRC checksum of packet
      if ((hedgehog_serial_buf[hedgehog_packet_size] == 0)&&(hedgehog_serial_buf[hedgehog_packet_size+1] == 0))
        {// checksum success

          switch(hedgehog_data_id)
          {
            case HEDGEHOG_POS_PACKET_ID:
            {
              // coordinates of hedgehog (X,Y), cm ==> mm
              un16.b[0]= hedgehog_serial_buf[9];
              un16.b[1]= hedgehog_serial_buf[10];
              hedgehog_x= 10*long(un16.wi);

              un16.b[0]= hedgehog_serial_buf[11];
              un16.b[1]= hedgehog_serial_buf[12];
              hedgehog_y= 10*long(un16.wi);
              
              // height of hedgehog, cm==>mm (FW V3.97+)
              un16.b[0]= hedgehog_serial_buf[13];
              un16.b[1]= hedgehog_serial_buf[14];
              hedgehog_z= 10*long(un16.wi);
              hedgehog_pos_updated= 1;// flag of new data from hedgehog received
              high_resolution_mode= false;
              break;
            }

            case HEDGEHOG_POS_HIGHRES_PACKET_ID:
            {
              // coordinates of hedgehog (X,Y), mm
              un32.b[0]= hedgehog_serial_buf[9];
              un32.b[1]= hedgehog_serial_buf[10];
              un32.b[2]= hedgehog_serial_buf[11];
              un32.b[3]= hedgehog_serial_buf[12];
              hedgehog_x= un32.vi32;

              un32.b[0]= hedgehog_serial_buf[13];
              un32.b[1]= hedgehog_serial_buf[14];
              un32.b[2]= hedgehog_serial_buf[15];
              un32.b[3]= hedgehog_serial_buf[16];
              hedgehog_y= un32.vi32;
              
              // height of hedgehog, mm 
              un32.b[0]= hedgehog_serial_buf[17];
              un32.b[1]= hedgehog_serial_buf[18];
              un32.b[2]= hedgehog_serial_buf[19];
              un32.b[3]= hedgehog_serial_buf[20];
              hedgehog_z= un32.vi32;
              
              hedgehog_pos_updated= 1;// flag of new data from hedgehog received
              high_resolution_mode= true;
              break;
            }    

            case BEACONS_POS_PACKET_ID:
            {
              for(i=0;i<MAX_BEACONS_SAVE;i++)
               {
                 beacon_pos[i].ok= 0;
               }
    
              n= hedgehog_serial_buf[5];
              if (n>MAX_BEACONS_SAVE) n= MAX_BEACONS_SAVE;

              for(i=0;i<n;i++)
               {
                 ofs= 6+i*8;
                 beacon_pos[i].adr= hedgehog_serial_buf[ofs+0];

                 un16.b[0]= hedgehog_serial_buf[ofs+1];
                 un16.b[1]= hedgehog_serial_buf[ofs+2];
                 beacon_pos[i].x= 10*long(un16.wi);

                 un16.b[0]= hedgehog_serial_buf[ofs+3];
                 un16.b[1]= hedgehog_serial_buf[ofs+4];
                 beacon_pos[i].y= 10*long(un16.wi);

                 un16.b[0]= hedgehog_serial_buf[ofs+5];
                 un16.b[1]= hedgehog_serial_buf[ofs+6];
                 beacon_pos[i].z= 10*long(un16.wi);
                 
                 beacon_pos[i].ok= 1;

                 beacons_pos_appeared= true;
               }
              break;
            }

            case BEACONS_POS_HIGHRES_PACKET_ID:
            {
              for(i=0;i<MAX_BEACONS_SAVE;i++)
               {
                 beacon_pos[i].ok= 0;
               }
    
              n= hedgehog_serial_buf[5];
              if (n>MAX_BEACONS_SAVE) n= MAX_BEACONS_SAVE;

              for(i=0;i<n;i++)
               {
                 ofs= 6+i*14;
                 beacon_pos[i].adr= hedgehog_serial_buf[ofs+0];

                 un32.b[0]= hedgehog_serial_buf[ofs+1];
                 un32.b[1]= hedgehog_serial_buf[ofs+2];
                 un32.b[2]= hedgehog_serial_buf[ofs+3];
                 un32.b[3]= hedgehog_serial_buf[ofs+4];
                 beacon_pos[i].x= un32.vi32;

                 un32.b[0]= hedgehog_serial_buf[ofs+5];
                 un32.b[1]= hedgehog_serial_buf[ofs+6];
                 un32.b[2]= hedgehog_serial_buf[ofs+7];
                 un32.b[3]= hedgehog_serial_buf[ofs+8];
                 beacon_pos[i].y= un32.vi32;

                 un32.b[0]= hedgehog_serial_buf[ofs+9];
                 un32.b[1]= hedgehog_serial_buf[ofs+10];
                 un32.b[2]= hedgehog_serial_buf[ofs+11];
                 un32.b[3]= hedgehog_serial_buf[ofs+12];
                 beacon_pos[i].z= un32.vi32;
                 
                 beacon_pos[i].ok= 1;

                 beacons_pos_appeared= true;
               }
              break;
            }
          }//switch(hedgehog_data_id)
        }// if CRC OK
    }//if (packet_received)  
}// void loop_hedgehog()

// Calculate CRC-16 of hedgehog packet
void hedgehog_set_crc16(byte *buf, byte size)
{uni_8x2_16 sum;
 byte shift_cnt;
 byte byte_cnt;

  sum.w=0xffffU;

  for(byte_cnt=size; byte_cnt>0; byte_cnt--)
   {
   sum.w=(unsigned int) ((sum.w/256U)*256U + ((sum.w%256U)^(buf[size-byte_cnt])));

     for(shift_cnt=0; shift_cnt<8; shift_cnt++)
       {
         if((sum.w&0x1)==1) sum.w=(unsigned int)((sum.w>>1)^0xa001U);
                       else sum.w>>=1;
       }
   }

  buf[size]=sum.b[0];
  buf[size+1]=sum.b[1];// little endian
}// hedgehog_set_crc16

//  END OF MARVELMIND HEDGEHOG RELATED PART
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


#define CM 1      //Centimeter
#define INC 0     //Inch
#define TP 2      //Trig_pin
#define EP 3      //Echo_pin

LiquidCrystal lcd(8, 13, 9, 4, 5, 6, 7);

// Output hedgehog range to LCD
void print_hedgehog_range(int lcd_x, int lcd_y, int beacon_index)
{long rl;
 char lcd_buf[12]; 
 byte lcd_coord_precision;
 
  lcd.setCursor(lcd_x,lcd_y); 
  if (beacon_pos[beacon_index].ok == 0)
    {
      lcd.print("        ");
      return;
    }

  if (high_resolution_mode)
    {
      lcd_coord_precision= 3;
    }
   else
    {
      lcd_coord_precision= 2; 
    }
  
  lcd.print(beacon_pos[beacon_index].adr);
  lcd.print("=");
  rl= hedgehog_range(beacon_index);
  dtostrf(((float) rl)/1000.0f, 4, lcd_coord_precision, lcd_buf);
  lcd.print(lcd_buf);
  lcd.print(" ");
}

void setup()
{
  lcd.clear(); 
  lcd.begin(16, 2);
  lcd.setCursor(0,0); 
  pinMode(TP,OUTPUT);       // set TP output for trigger  
  pinMode(EP,INPUT);        // set EP input for echo
  
  lcd.setCursor(0,0); 
  lcd.print("Waiting");
  lcd.setCursor(0,1); 
  lcd.print("positions");
  
  setup_hedgehog();//    Marvelmind hedgehog support initialize
}

void loop()
{

#ifdef DISTANCE_SENSOR_ENABLED
   long microseconds = TP_init();
   long distacne_cm = Distance(microseconds, CM);
   //lcd.setCursor(10,0); 
   //lcd.print("D="); 
   //lcd.print(distacne_cm); 
   //lcd.print("  "); 
#endif

   loop_hedgehog();// Marvelmind hedgehog service loop

   if ((hedgehog_pos_updated)&&(beacons_pos_appeared))
     {// new data from hedgehog available
       hedgehog_pos_updated= 0;// clear new data flag 
       
       // output hedgehog ranges to LCD
       print_hedgehog_range(0,0, 0);
       print_hedgehog_range(8,0, 1);
       print_hedgehog_range(0,1, 2);
       print_hedgehog_range(8,1, 3); 
     }
}

long Distance(long time, int flag)
{
  /*
  
  */
  long distacne;
  if(flag)
    distacne = time /29 / 2  ;     // Distance_CM  = ((Duration of high level)*(Sonic :340m/s))/2
                                   //              = ((Duration of high level)*(Sonic :0.034 cm/us))/2
                                   //              = ((Duration of high level)/(Sonic :29.4 cm/us))/2
  else
    distacne = time / 74 / 2;      // INC
  return distacne;
}

long TP_init()
{                     
  digitalWrite(TP, LOW);                    
  delayMicroseconds(2);
  digitalWrite(TP, HIGH);                 // pull the Trig pin to high level for more than 10us impulse 
  delayMicroseconds(10);
  digitalWrite(TP, LOW);
  long microseconds = pulseIn(EP,HIGH);   // waits for the pin to go HIGH, and returns the length of the pulse in microseconds
  return microseconds;                    // return microseconds
}









