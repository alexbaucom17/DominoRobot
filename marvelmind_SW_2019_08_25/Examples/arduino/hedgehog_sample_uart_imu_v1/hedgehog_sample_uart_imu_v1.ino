/*
  The circuit:
 * HEDGEHOG serial data to digital pin 0 (RXD)
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

// Modes of LCD data output
#define LCD_RAW_GYRO 0
#define LCD_RAW_COMPASS 1
#define LCD_FUSION_QUATERNION 2
#define LCD_FUSION_V 3
#define LCD_FUSION_A 4

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//  MARVELMIND HEDGEHOG RELATED PART

long hedgehog_x, hedgehog_y;// coordinates of hedgehog (X,Y), mm
long hedgehog_z;// height of hedgehog, mm
int hedgehog_pos_updated;// flag of new data from hedgehog received

bool high_resolution_mode;

// IMU sensors raw data
int16_t imu_acc_x,imu_acc_y,imu_acc_z;
int16_t imu_gyro_x,imu_gyro_y,imu_gyro_z;
int16_t imu_compass_x,imu_compass_y,imu_compass_z;
uint32_t imu_raw_timestamp;
int imu_raw_updated;

// IMU fusion data
int32_t imu_fusion_x, imu_fusion_y, imu_fusion_z;// imu fusion position
int16_t imu_fusion_qw, imu_fusion_qx, imu_fusion_qy, imu_fusion_qz;// quaternion
int16_t imu_fusion_vx, imu_fusion_vy, imu_fusion_vz;// imu fusion speed
int16_t imu_fusion_ax, imu_fusion_ay, imu_fusion_az;//imu fusion acceleration
uint32_t imu_fusion_timestamp;
int imu_fusion_updated;

///

#define HEDGEHOG_BUF_SIZE 50 
#define HEDGEHOG_CM_DATA_SIZE 0x10
#define HEDGEHOG_MM_DATA_SIZE 0x16
#define HEDGEHOG_RAW_IMU_DATA_SIZE 0x20
#define HEDGEHOG_IMU_FUSION_DATA_SIZE 0x2A
byte hedgehog_serial_buf[HEDGEHOG_BUF_SIZE];
byte hedgehog_serial_buf_ofs;

#define POSITION_DATAGRAM_ID 0x0001
#define POSITION_DATAGRAM_HIGHRES_ID 0x0011
#define RAW_IMU_DATAGRAM_ID 0x0003
#define IMU_FUSION_DATAGRAM_ID 0x0005
unsigned int hedgehog_data_id;

typedef union {byte b[2]; unsigned int w;int wi;} uni_8x2_16;
typedef union {byte b[4];float f;unsigned long v32;long vi32;} uni_8x4_32;

//    Marvelmind hedgehog support initialize
void setup_hedgehog() 
{
  Serial.begin(500000); // hedgehog transmits data on 500 kbps  

  hedgehog_serial_buf_ofs= 0;
  hedgehog_pos_updated= 0;

  imu_raw_updated= 0;
  imu_fusion_updated= 0;
}

// Marvelmind hedgehog service loop
void loop_hedgehog()
{int incoming_byte;
 int total_received_in_loop;
 int packet_received;
 bool good_byte;
 byte packet_size;
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
      if (total_received_in_loop>100) break;// too much data without required header
      
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
          good_byte=   (hedgehog_data_id == POSITION_DATAGRAM_ID) ||
                       (hedgehog_data_id == POSITION_DATAGRAM_HIGHRES_ID) ||
                       (hedgehog_data_id == RAW_IMU_DATAGRAM_ID) ||
                       (hedgehog_data_id == IMU_FUSION_DATAGRAM_ID);
          break;
        }
        case 4:
        {
          switch(hedgehog_data_id)
          {
            case POSITION_DATAGRAM_ID:
            {
              good_byte= (incoming_byte == HEDGEHOG_CM_DATA_SIZE);
              break;
            }
            case POSITION_DATAGRAM_HIGHRES_ID:
            {
              good_byte= (incoming_byte == HEDGEHOG_MM_DATA_SIZE);
              break;
            }
            case RAW_IMU_DATAGRAM_ID:
            {
              good_byte= (incoming_byte == HEDGEHOG_RAW_IMU_DATA_SIZE);
              break;
            }
            case IMU_FUSION_DATAGRAM_ID:
            {
              good_byte= (incoming_byte == HEDGEHOG_IMU_FUSION_DATA_SIZE);
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
          continue;
        }     
      hedgehog_serial_buf[hedgehog_serial_buf_ofs++]= incoming_byte; 
      if (hedgehog_serial_buf_ofs>5)
        {
          packet_size=  7 + hedgehog_serial_buf[4];
          if (hedgehog_serial_buf_ofs == packet_size)
            {// received packet with required header
              packet_received= 1;
              hedgehog_serial_buf_ofs= 0;// restart bufer fill
              break; 
            }
        }
    }

  if (packet_received)  
    {
      hedgehog_set_crc16(&hedgehog_serial_buf[0], packet_size);// calculate CRC checksum of packet
      if ((hedgehog_serial_buf[packet_size] == 0)&&(hedgehog_serial_buf[packet_size+1] == 0))
        {// checksum success
          switch(hedgehog_data_id)
          {
            case POSITION_DATAGRAM_ID:
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
              return;
              break;
            }

            case POSITION_DATAGRAM_HIGHRES_ID:
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
              return;
              break;
            }

            case RAW_IMU_DATAGRAM_ID:
            {
              un16.b[0]= hedgehog_serial_buf[5];
              un16.b[1]= hedgehog_serial_buf[6];
              imu_acc_x= un16.wi;

              un16.b[0]= hedgehog_serial_buf[7];
              un16.b[1]= hedgehog_serial_buf[8];
              imu_acc_y= un16.wi;

              un16.b[0]= hedgehog_serial_buf[9];
              un16.b[1]= hedgehog_serial_buf[10];
              imu_acc_z= un16.wi;

              //

              un16.b[0]= hedgehog_serial_buf[11];
              un16.b[1]= hedgehog_serial_buf[12];
              imu_gyro_x= un16.wi;

              un16.b[0]= hedgehog_serial_buf[13];
              un16.b[1]= hedgehog_serial_buf[14];
              imu_gyro_y= un16.wi;

              un16.b[0]= hedgehog_serial_buf[15];
              un16.b[1]= hedgehog_serial_buf[16];
              imu_gyro_z= un16.wi;

              //

              un16.b[0]= hedgehog_serial_buf[17];
              un16.b[1]= hedgehog_serial_buf[18];
              imu_compass_x= un16.wi;

              un16.b[0]= hedgehog_serial_buf[19];
              un16.b[1]= hedgehog_serial_buf[20];
              imu_compass_y= un16.wi;

              un16.b[0]= hedgehog_serial_buf[21];
              un16.b[1]= hedgehog_serial_buf[22];
              imu_compass_z= un16.wi; 

              un32.b[0]= hedgehog_serial_buf[29];
              un32.b[1]= hedgehog_serial_buf[30];
              un32.b[2]= hedgehog_serial_buf[31];
              un32.b[3]= hedgehog_serial_buf[32];
              imu_raw_timestamp= un32.vi32;

              imu_raw_updated= 1;
              return;
              break;
            }

            case IMU_FUSION_DATAGRAM_ID:
            {
              un32.b[0]= hedgehog_serial_buf[5+0];
              un32.b[1]= hedgehog_serial_buf[5+1];
              un32.b[2]= hedgehog_serial_buf[5+2];
              un32.b[3]= hedgehog_serial_buf[5+3];
              imu_fusion_x= un32.vi32;

              un32.b[0]= hedgehog_serial_buf[9+0];
              un32.b[1]= hedgehog_serial_buf[9+1];
              un32.b[2]= hedgehog_serial_buf[9+2];
              un32.b[3]= hedgehog_serial_buf[9+3];
              imu_fusion_y= un32.vi32;

              un32.b[0]= hedgehog_serial_buf[13+0];
              un32.b[1]= hedgehog_serial_buf[13+1];
              un32.b[2]= hedgehog_serial_buf[13+2];
              un32.b[3]= hedgehog_serial_buf[13+3];
              imu_fusion_z= un32.vi32;

              //////

              un16.b[0]= hedgehog_serial_buf[17+0];
              un16.b[1]= hedgehog_serial_buf[17+1];
              imu_fusion_qw= un16.wi;

              un16.b[0]= hedgehog_serial_buf[19+0];
              un16.b[1]= hedgehog_serial_buf[19+1];
              imu_fusion_qx= un16.wi;

              un16.b[0]= hedgehog_serial_buf[21+0];
              un16.b[1]= hedgehog_serial_buf[21+1];
              imu_fusion_qy= un16.wi;

              un16.b[0]= hedgehog_serial_buf[23+0];
              un16.b[1]= hedgehog_serial_buf[23+1];
              imu_fusion_qz= un16.wi;

              //////

              un16.b[0]= hedgehog_serial_buf[25+0];
              un16.b[1]= hedgehog_serial_buf[25+1];
              imu_fusion_vx= un16.wi;

              un16.b[0]= hedgehog_serial_buf[27+0];
              un16.b[1]= hedgehog_serial_buf[27+1];
              imu_fusion_vy= un16.wi;

              un16.b[0]= hedgehog_serial_buf[29+0];
              un16.b[1]= hedgehog_serial_buf[29+1];
              imu_fusion_vz= un16.wi;

              //////

              un16.b[0]= hedgehog_serial_buf[31+0];
              un16.b[1]= hedgehog_serial_buf[31+1];
              imu_fusion_ax= un16.wi;

              un16.b[0]= hedgehog_serial_buf[33+0];
              un16.b[1]= hedgehog_serial_buf[33+1];
              imu_fusion_ay= un16.wi;

              un16.b[0]= hedgehog_serial_buf[35+0];
              un16.b[1]= hedgehog_serial_buf[35+1];
              imu_fusion_az= un16.wi;

              //////

              un32.b[0]= hedgehog_serial_buf[39+0];
              un32.b[1]= hedgehog_serial_buf[39+1];
              un32.b[2]= hedgehog_serial_buf[39+2];
              un32.b[3]= hedgehog_serial_buf[39+3];
              imu_raw_timestamp= un32.vi32;
              
              imu_fusion_updated= 1;
              return;
              break;
            }
          }//switch(hedgehog_data_id)
        }// checksum success 
    }//if (packet_received)  
}//void loop_hedgehog()

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
uint8_t lcd_show_mode;

void setup()
{
  lcd.clear(); 
  lcd.begin(16, 2);

  lcd.setCursor(0,0); 
  lcd.print("Waiting");
  lcd.setCursor(0,1); 
  lcd.print("IMU data...");

//#define LCD_RAW_GYRO 0
//#define LCD_RAW_COMPASS 1
//#define LCD_FUSION_QUATERNION 2
//#define LCD_FUSION_V 3
//#define LCD_FUSION_A 4
  lcd_show_mode= LCD_RAW_COMPASS;

  setup_hedgehog();//    Marvelmind hedgehog support initialize
}

void loop()
{  byte lcd_coord_precision;
   char lcd_buf[12];

   loop_hedgehog();// Marvelmind hedgehog service loop

   if (imu_raw_updated)
     {// new raw IMU data from hedgehog available
       imu_raw_updated= 0;// clear new data flag 

       if ((lcd_show_mode == LCD_RAW_GYRO)||(lcd_show_mode == LCD_RAW_COMPASS))
       {
           // First line - accelerometer data
           lcd.setCursor(0,0); 
           itoa(imu_acc_x, &lcd_buf[0], 10);
           lcd.print(lcd_buf);
           lcd.print("    ");  
           
           lcd.setCursor(5,0);
           itoa(imu_acc_y, &lcd_buf[0], 10);
           lcd.print(lcd_buf);
           lcd.print("    ");  
           
           lcd.setCursor(10,0);
           itoa(imu_acc_z, &lcd_buf[0], 10);
           lcd.print(lcd_buf);
           lcd.print("    ");  
    
           if (lcd_show_mode == LCD_RAW_GYRO)
           {
              // Second line - gyro data
              lcd.setCursor(0,1); 
              itoa(imu_gyro_x, &lcd_buf[0], 10);
              lcd.print(lcd_buf);
              lcd.print("    ");  
            
              lcd.setCursor(5,1);
              itoa(imu_gyro_y, &lcd_buf[0], 10);
              lcd.print(lcd_buf);
              lcd.print("    ");  
           
              lcd.setCursor(10,1);
              itoa(imu_gyro_z, &lcd_buf[0], 10);
              lcd.print(lcd_buf);
              lcd.print("    ");
           }
           else
           {                  
               // Second line - compass data
               lcd.setCursor(0,1); 
               itoa(imu_compass_x, &lcd_buf[0], 10);
               lcd.print(lcd_buf);
               lcd.print("    ");  
                
               lcd.setCursor(5,1);
               itoa(imu_compass_y, &lcd_buf[0], 10);
               lcd.print(lcd_buf);
               lcd.print("    ");  
               
               lcd.setCursor(10,1);
               itoa(imu_compass_z, &lcd_buf[0], 10);
               lcd.print(lcd_buf);
           }
       }
       //delay(200);
     }

     if (imu_fusion_updated) 
     {
       imu_fusion_updated= 0;

       if ((lcd_show_mode == LCD_FUSION_QUATERNION)||(lcd_show_mode == LCD_FUSION_V) ||(lcd_show_mode == LCD_FUSION_A))
       {
           // First line - fusion position data
           lcd.setCursor(0,0); 
           itoa(imu_fusion_x, &lcd_buf[0], 10);
           lcd.print(lcd_buf);
           lcd.print("    ");  
           
           lcd.setCursor(5,0);
           itoa(imu_fusion_y, &lcd_buf[0], 10);
           lcd.print(lcd_buf);
           lcd.print("    ");  
           
           lcd.setCursor(10,0);
           itoa(imu_fusion_z, &lcd_buf[0], 10);
           lcd.print(lcd_buf);
           lcd.print("    ");  
    
           if (lcd_show_mode == LCD_FUSION_QUATERNION)
           {
              // Second line - quaternion data
              lcd.setCursor(0,1); 
              itoa(imu_fusion_qx, &lcd_buf[0], 10);
              lcd.print(lcd_buf);
              lcd.print("    ");  
            
              lcd.setCursor(5,1);
              itoa(imu_fusion_qy, &lcd_buf[0], 10);
              lcd.print(lcd_buf);
              lcd.print("    ");  
           
              lcd.setCursor(10,1);
              itoa(imu_fusion_qz, &lcd_buf[0], 10);
              lcd.print(lcd_buf);
              lcd.print("    ");
           }
           else if (lcd_show_mode == LCD_FUSION_V)
           {                  
               // Second line - speed data
               lcd.setCursor(0,1); 
               itoa(imu_fusion_vx, &lcd_buf[0], 10);
               lcd.print(lcd_buf);
               lcd.print("    ");  
                
               lcd.setCursor(5,1);
               itoa(imu_fusion_vy, &lcd_buf[0], 10);
               lcd.print(lcd_buf);
               lcd.print("    ");  
               
               lcd.setCursor(10,1);
               itoa(imu_fusion_vz, &lcd_buf[0], 10);
               lcd.print(lcd_buf);
           }
           else if (lcd_show_mode == LCD_FUSION_A)
           {
                // Second line - accelerometer data
               lcd.setCursor(0,1); 
               itoa(imu_fusion_ax, &lcd_buf[0], 10);
               lcd.print(lcd_buf);
               lcd.print("    ");  
                
               lcd.setCursor(5,1);
               itoa(imu_fusion_ay, &lcd_buf[0], 10);
               lcd.print(lcd_buf);
               lcd.print("    ");  
               
               lcd.setCursor(10,1);
               itoa(imu_fusion_az, &lcd_buf[0], 10);
               lcd.print(lcd_buf);           
           }
       }
     }
}









