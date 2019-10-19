/*
 *   This example reads position and waypoints from mobile beacon (hedgehog)
 */

/*
  The circuit:
 * Serial data from hedgehog : digital pin 0 (RXD)
 * Serial data to hedgehog : digital pin 1 (TXD)
 * Button "ready to receive path" : digital pin 3. Should be shorted to GND if device is ready to receive
 * LCD RS pin : digital pin 8
 * LCD Enable pin : digital pin 9
 * LCD D4 pin : digital pin 4
 * LCD D5 pin : digital pin 5
 * LCD D6 pin : digital pin 6
 * LCD D7 pin : digital pin 7
 * LCD BL pin : digital pin 10
 *Vcc pin :  +5
 */

 /*
  *   Some comments:
  *   Digital pin 3 has dual function:
  *   1. If connected to GND it allows loading the movement path into the device
  *   2. After loading of the path if pin 3 is connected to GND, first line of LCD shows the movement path step by step  with 1 sec interval
  */

#include <LiquidCrystal.h>

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//  MARVELMIND HEDGEHOG RELATED PART

#define READY_RECEIVE_PATH_PIN 3

//////////////////

#define PACKET_TYPE_STREAM_FROM_HEDGE 0x47
#define PACKET_TYPE_REPLY_TO_STREAM 0x48
#define PACKET_TYPE_READ_FROM_DEVICE 0x49
#define PACKET_TYPE_WRITE_TO_DEVICE 0x4a

#define HEDGEHOG_POS_PACKET_ID 0x0001
#define READY_CONFIRM_PACKET_ID 0x0100
#define MOVEMENT_PATH_PACKET_ID 0x0201

#define DATA_OFS 5

int hedgehog_x, hedgehog_y;// coordinates of hedgehog (X,Y), cm
int hedgehog_z;// height of hedgehog, cm
byte hedgehog_flags= 0;
byte hedgehog_address= 0;
int hedgehog_pos_updated;// flag of new data from hedgehog received

///

#define HEDGEHOG_BUF_SIZE 64 
byte hedgehog_serial_buf[HEDGEHOG_BUF_SIZE];
byte hedgehog_serial_buf_ofs;
byte hedgehog_packet_size;

byte hedgehog_packet_type;
int hedgehog_packet_id;

typedef union {byte b[2]; unsigned int w;} uni_8x2_16;

typedef struct {
  byte moveType;
  int param1;
  int param2;
} MoveItem;
#define MAX_MOVE_ITEMS 64
MoveItem moveItems[MAX_MOVE_ITEMS];
byte moveItemsNum= 0;

unsigned long prevMoveItemShowTime= 0;
byte moveItemShowIndex= 0;

////////////////////////////////////////////////////////////////////////////

//    Marvelmind hedgehog support initialize
void setup_hedgehog() 
{int i;
  Serial.begin(19200); // hedgehog transmits data on 500 kbps  

  hedgehog_serial_buf_ofs= 0;
  hedgehog_pos_updated= 0;

  hedgehog_packet_id= 0;

  pinMode(READY_RECEIVE_PATH_PIN,INPUT_PULLUP);        // set EP input for echo
}

////////////////////////////////////////

void hedgehog_send_packet(byte address, byte packet_type, unsigned int id, byte data_size)
{byte frameSizeBeforeCRC;

   hedgehog_serial_buf[0]= address;

   hedgehog_serial_buf[1]= packet_type;

   hedgehog_serial_buf[2]= id&0xff;
   hedgehog_serial_buf[3]= (id>>8)&0xff;

   if (data_size != 0)
   {
     hedgehog_serial_buf[4]= data_size;
     frameSizeBeforeCRC= data_size+5;
   }  
   else
   {
     frameSizeBeforeCRC= 4;
   }

   hedgehog_set_crc16(&hedgehog_serial_buf[0], frameSizeBeforeCRC);

   Serial.write(hedgehog_serial_buf, frameSizeBeforeCRC+2);
}

// Sends confirmation packet of readyness state to transmit/receive data 
void hedgehog_send_ready_confirm()
{byte status= 0;

  //if (digitalRead(READY_RECEIVE_PATH_PIN)== LOW)
    status|= (1<<0);// ready to receive data 
  hedgehog_serial_buf[DATA_OFS + 0]= status;
  hedgehog_serial_buf[DATA_OFS + 1]= 0;
  hedgehog_serial_buf[DATA_OFS + 2]= 0;
  hedgehog_serial_buf[DATA_OFS + 3]= 0;
  
  hedgehog_send_packet(hedgehog_address, PACKET_TYPE_REPLY_TO_STREAM, READY_CONFIRM_PACKET_ID, 4); 
}

// Sends answer on write request
void hedgehog_send_write_answer_success()
{
   hedgehog_send_packet(hedgehog_address, hedgehog_packet_type, hedgehog_packet_id, 0); 
}

////////////////////////////////////////

void restart_packet_receive()
{
   hedgehog_serial_buf_ofs= 0;// restart bufer fill
   hedgehog_packet_id= 0;    
}

int check_packet_data_size(byte packet_type, unsigned int packet_id, byte size, byte wanted_size)
{
   if (hedgehog_packet_type == packet_type) 
     if (hedgehog_packet_id == packet_id)
       if (size != wanted_size)
         {
           restart_packet_receive(); 
           return 0;
         }
   return 1;  
}

void process_stream_packet()
{
   if (hedgehog_packet_id == HEDGEHOG_POS_PACKET_ID)
      {// packet of hedgehog position
         hedgehog_x= int(hedgehog_serial_buf[9]) + (int(hedgehog_serial_buf[10])<<8);
         hedgehog_y= int(hedgehog_serial_buf[11]) + (int(hedgehog_serial_buf[12])<<8);// coordinates of hedgehog (X,Y), cm
         hedgehog_z= int(hedgehog_serial_buf[13]) + (int(hedgehog_serial_buf[14])<<8);// height of hedgehog, cm (FW V3.97+)
         hedgehog_flags= hedgehog_serial_buf[15];
         hedgehog_address= hedgehog_serial_buf[16];
         hedgehog_pos_updated= 1;// flag of new data from hedgehog received

         if (hedgehog_flags&0x08)
          {// request for writing data
            hedgehog_send_ready_confirm();
          }
      }  
}

void process_write_packet()
{
  if (hedgehog_packet_id == MOVEMENT_PATH_PACKET_ID)
     {// movement path packet
        byte indcur= hedgehog_serial_buf[5+1]; 
        moveItemsNum= hedgehog_serial_buf[5+2]; 

        if (moveItemsNum>=MAX_MOVE_ITEMS) 
          moveItemsNum= MAX_MOVE_ITEMS;
        if (indcur<moveItemsNum) 
          {
            moveItems[indcur].moveType= hedgehog_serial_buf[5+0];
            moveItems[indcur].param1= hedgehog_serial_buf[5+3] | (int(hedgehog_serial_buf[5+4])<<8);
            moveItems[indcur].param2= hedgehog_serial_buf[5+5] | (int(hedgehog_serial_buf[5+6])<<8);

            prevMoveItemShowTime= millis();
          }

        lcd_show_path_progress(indcur,moveItemsNum);

        // send answer packet
        hedgehog_send_write_answer_success();
     }
}

// Marvelmind hedgehog service loop
void loop_hedgehog()
{int incoming_byte;
 int total_received_in_loop;
 int packet_received;
 int packet_id;
 int i,n,ofs;

  total_received_in_loop= 0;
  packet_received= 0;
  while(Serial.available() > 0)
    {
      if (hedgehog_serial_buf_ofs>=HEDGEHOG_BUF_SIZE) 
        {
          hedgehog_serial_buf_ofs= 0;
          break;// buffer overflow
        }
      total_received_in_loop++;
      if (total_received_in_loop>100) break;// too much data without required header
      
      incoming_byte= Serial.read();
  
      if (hedgehog_serial_buf_ofs==0)
        {// check first bytes for constant value
          if (incoming_byte != 0xff) 
            {
              hedgehog_serial_buf_ofs= 0;// restart bufer fill
              hedgehog_packet_id= 0;
              continue;
            }
        } 
      else if (hedgehog_serial_buf_ofs==1)
        {// check packet type
          if ( (incoming_byte == PACKET_TYPE_STREAM_FROM_HEDGE) ||
               (incoming_byte == PACKET_TYPE_READ_FROM_DEVICE) ||
               (incoming_byte == PACKET_TYPE_WRITE_TO_DEVICE) 
             )
            {// correct packet type - save  
              hedgehog_packet_type= incoming_byte; 
            }
           else
            {// incorrect packet type - skip packet
              restart_packet_receive();
              continue;            
            }
        }
      else if (hedgehog_serial_buf_ofs==3) 
        {// Check two-bytes packet ID
          hedgehog_packet_id= hedgehog_serial_buf[2] + incoming_byte*256;
          switch(hedgehog_packet_type)
          {
            case PACKET_TYPE_STREAM_FROM_HEDGE:
              {
                switch(hedgehog_packet_id)
                  {
                    case HEDGEHOG_POS_PACKET_ID:
                      {
                        break;
                      }

                    default:
                      {// incorrect packet ID - skip packet
                        restart_packet_receive();
                        continue;                        
                      }
                  }
                break;
              }

              case PACKET_TYPE_WRITE_TO_DEVICE:
              {
                switch(hedgehog_packet_id)
                  {
                    case MOVEMENT_PATH_PACKET_ID:
                      {                     
                        break;
                      }

                    default:
                      {// incorrect packet ID - skip packet
                        restart_packet_receive();
                        continue;                        
                      }
                  }
                break;
              }
          }        
        }
    else if (hedgehog_serial_buf_ofs==4) 
      {// data field size
        if (check_packet_data_size(PACKET_TYPE_STREAM_FROM_HEDGE, HEDGEHOG_POS_PACKET_ID, incoming_byte, 0x10) == 0)
           continue;// incorrect hedgehog coordinates data size

        if (check_packet_data_size(PACKET_TYPE_WRITE_TO_DEVICE, MOVEMENT_PATH_PACKET_ID, incoming_byte, 0x0c) == 0)
           continue;// incorrect movement path packet data size
           
         // save required packet size   
         hedgehog_packet_size= incoming_byte + 7;
      }
        
      hedgehog_serial_buf[hedgehog_serial_buf_ofs++]= incoming_byte; 
      if (hedgehog_serial_buf_ofs>5) 
       if (hedgehog_serial_buf_ofs == hedgehog_packet_size)
        {// received packet with required header
          packet_received= 1;
          hedgehog_serial_buf_ofs= 0;// restart bufer fill
          break; 
        }
    }
    

  if (packet_received)  
    {
      hedgehog_set_crc16(&hedgehog_serial_buf[0], hedgehog_packet_size);// calculate CRC checksum of packet
      if ((hedgehog_serial_buf[hedgehog_packet_size] == 0)&&(hedgehog_serial_buf[hedgehog_packet_size+1] == 0))
        {// checksum success
          switch(hedgehog_packet_type)
          {
            case PACKET_TYPE_STREAM_FROM_HEDGE:
             {
               process_stream_packet();
               break;        
             }     
          
            case PACKET_TYPE_WRITE_TO_DEVICE:
             {            
               process_write_packet();
               break;
             }
          }//switch  
        }
    }// if (packet_received)  
}// loop_hedgehog

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

LiquidCrystal lcd(8, 13, 9, 4, 5, 6, 7);

void lcd_show_path_progress(byte indcur, byte ntotal)
{
  lcd.setCursor(8,0); 
  lcd.print("P=");  
  lcd.print(indcur+1);
  lcd.print('/');
  lcd.print(ntotal);
  lcd.print("  "); 
}

void lcd_show_move_item()
{
  lcd.setCursor(0,0); 
  lcd.print(moveItemShowIndex);
  lcd.print(':');

  byte moveType= moveItems[moveItemShowIndex].moveType;
  int param1= moveItems[moveItemShowIndex].param1;
  int param2= moveItems[moveItemShowIndex].param2;
  char c;
  switch(moveType)
  {
    case 0: c= 'F'; break;
    case 1: c= 'B'; break;
    case 2: c= 'R'; break;
    case 3: c= 'L'; break;
    case 4: c= 'P'; break;
    case 5: c= 'G'; break;
    case 6: c= 'M'; break;
    case 7: c= 'V'; break;
    default: c='?';break;
  }
  lcd.print(c);
  switch(moveType)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4: 
    case 6:
    case 7: {
      lcd.print(param1);
      break;
    }
  }
  switch(moveType)
  {
    case 6: {
      lcd.print(',');
      lcd.print(param2);
      break;
    }
  } 
  lcd.print("        ");   
}

void setup()
{
  lcd.clear(); 
  lcd.begin(16, 2);
  lcd.setCursor(0,0); 

  setup_hedgehog();//    Marvelmind hedgehog support initialize
}

void loop()
{bool showPath;

   delayMicroseconds(500);
   
   loop_hedgehog();// Marvelmind hedgehog service loop

   showPath= ((moveItemsNum != 0));// && (digitalRead(READY_RECEIVE_PATH_PIN)== LOW));

   if (hedgehog_pos_updated)
     {// new data from hedgehog available
       hedgehog_pos_updated= 0;// clear new data flag 

       if (!showPath)
         {      
           lcd.setCursor(0,0); 
           lcd.print("X=");
           lcd.print(hedgehog_x);
           lcd.print("   ");  
         }
       lcd.setCursor(0,1);
       lcd.print("Y=");
       lcd.print(hedgehog_y);  
       lcd.print("   ");  
       
       lcd.setCursor(10,1); 
       lcd.print("Z=");  
       lcd.print(hedgehog_z);
       lcd.print("  "); 
     }

  if (showPath)
     {
        unsigned long newMillis= millis();
        if (newMillis>prevMoveItemShowTime+1000)
          {
            lcd_show_move_item();
            
            moveItemShowIndex= (moveItemShowIndex + 1)%moveItemsNum;
            prevMoveItemShowTime= newMillis;
          }
     }
}










