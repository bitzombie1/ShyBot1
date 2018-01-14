/*
 * Sculpture 4 . Controls one 6v actuator for X-axis Low 90, High 140. Also the 12v AX-12 servo was thrown on the 6v 
 * rail to run smoother. 
 */
#include <Servo.h>
// for Dinamixel control
#define AX_START                    255
#define AX_ID                       3
#define AX_ID_LENGTH                4
#define AX_TORQUE_LENGTH            4
#define AX_WRITE_DATA               3
#define AX_TORQUE_ENABLE            24
#define AX_LED_LENGTH               4
#define AX_LED                      25
#define TX_DELAY_TIME               400 
#define AX_GOAL_LENGTH              5
#define AX_GOAL_SP_LENGTH           7
#define AX_GOAL_POSITION_L          30
#define AX_VL_LENGTH                6
#define AX_BD_LENGTH                4
#define AX_BAUD_RATE                4
#define sendData(args)  (Serial.write(args))    // Write Over Serial
#define delayus(args) (delayMicroseconds(args))  // Delay Microseconds


#define BUFFSIZE 32

#define UPBOARDONPIN A7       //reads 3.3v on UP-board to see if booted up
#define LEDSWPIN 2            // switches mosfet control for LEDs
// servo drive pins **************
#define SERVOX_PIN 11

char inString[BUFFSIZE];          // Serial command buffer
bool serialEcho = true;          // do, or do not, repeat after me 
unsigned char Checksum;

Servo servo_x ;
int pos_lst[] = {200,300,400,500,600,700,800};    //where our ax-12 servo can move to

bool beat = true;                //debug LED to know we are still looping 
int curr_z =1900; 

void setup() {
  Serial.begin(1000000);  // 115200 Dynamixel.begin(1000000,2);  // start the AX-12 servo at 1Mbps 
  delay(1000);
  //lower the baud rate on the motor
  dxSetBD(1,115200);
  Serial.end();
  delay(500);
  Serial.begin(115200);  //reconnect to serial
  //dxSetID(1,2);           this sets up ID in EEPROM run once

  // servo connect setup ***********************
  servo_x.attach(SERVOX_PIN);

  pinMode(LEDSWPIN,OUTPUT);
  pinMode(13,OUTPUT);
}

void loop() {
  if(serialCheck()){ commandParser(inString);}
  
  if(analogRead(UPBOARDONPIN) > 400){digitalWrite(LEDSWPIN, HIGH);}
  else{digitalWrite(LEDSWPIN, LOW); }
    
 
  if((curr_z < 1000) && millis() % 2000 == 0){ dxMoveSpeed(1,pos_lst[int(random(0, 7))],60);}
}

// Dinamixel AX-12 servo functions *************************************
void dxSetBD(int ID, long baud){

  unsigned char Baud_Rate = (2000000/baud) - 1;
  Checksum = (~(ID + AX_BD_LENGTH + AX_WRITE_DATA + AX_BAUD_RATE + Baud_Rate))&0xFF;
  
   sendData(AX_START);              // Send Instructions over Serial
   sendData(AX_START);
   sendData(ID);
   sendData(AX_BD_LENGTH);
   sendData(AX_WRITE_DATA);
   sendData(AX_BAUD_RATE);
   sendData(Baud_Rate);
   sendData(Checksum);
   delayus(TX_DELAY_TIME);
}

void dxledStatus(int ID, bool Status){    //id is motor ID, status is on or off 1 or 0
  Checksum = (~(ID + AX_LED_LENGTH + AX_WRITE_DATA + AX_LED + Status))&0xFF;
  
    sendData(AX_START);              // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
    sendData(AX_TORQUE_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_TORQUE_ENABLE);
    sendData(Status);
    sendData(Checksum);
    delayus(TX_DELAY_TIME);
}

void dxMove(int ID, int Position){
  char Position_H,Position_L;
  Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
  Position_L = Position;
  Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H))&0xFF;
    
    sendData(AX_START);                 // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
    sendData(AX_GOAL_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_GOAL_POSITION_L);
    sendData(Position_L);
    sendData(Position_H);
    sendData(Checksum);
  delayus(TX_DELAY_TIME);
}

void dxMoveSpeed(unsigned char ID, int Position, int Speed)
{
    char Position_H,Position_L,Speed_H,Speed_L;
    Position_H = Position >> 8;    
    Position_L = Position;                // 16 bits - 2 x 8 bits variables
    Speed_H = Speed >> 8;
    Speed_L = Speed;                      // 16 bits - 2 x 8 bits variables
  Checksum = (~(ID + AX_GOAL_SP_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H))&0xFF;
 
    sendData(AX_START);                // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
    sendData(AX_GOAL_SP_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_GOAL_POSITION_L);
    sendData(Position_L);
    sendData(Position_H);
    sendData(Speed_L);
    sendData(Speed_H);
    sendData(Checksum);
    delayus(TX_DELAY_TIME);   
}

void dxSetID(unsigned char ID, unsigned char newID)
{    
  Checksum = (~(ID + AX_ID_LENGTH + AX_WRITE_DATA + AX_ID + newID))&0xFF;

    sendData(AX_START);                // Send Instructions over Serial
    sendData(AX_START);
    sendData(ID);
    sendData(AX_ID_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_ID);
    sendData(newID);
    sendData(Checksum);
  delayus(TX_DELAY_TIME); 
}


// our regular serial functions *****************************
bool serialCheck() {
  int i=0;
  if (Serial.available()) {
    delay(50);
    while(Serial.available()) {
      inString[i++] = Serial.read();
    }
    inString[i++]='\0';  

  digitalWrite(13,HIGH);
    
    if (serialEcho) {
      Serial.println(inString);
    }
    //commandParser(inString);
    return true;
  }
  digitalWrite(13,LOW);
  return false;
}

void commandParser(char * command){
  int i =0;
  if(command[i] == 'h'){delay(1);} // home command
  else if(command[i] == 'q'){delay(1);}
  else if(command[i] == 's'){delay(1);}
  else if(command[i] == 'g'){
    char one_str[11]={}; int x =0; long one_l = 0;
    char two_str[11]={}; int y =0; long two_l =0;
    char three_str[11]={};  int z =0; long three_l =0;
    while(command[i] != '/' && i<BUFFSIZE) {
      i++;
      one_str[x] = command[i];
      x++;
    }
    one_str[x+1] = '\0'; 
    one_l = atol(one_str);
       
    i++;  // skip the forward slash
    
    while(command[i] != '/' && i<BUFFSIZE) {    
      two_str[y] = command[i];
      i++;
      y++;
    }
    two_str[y+1] = '\0'; 
    two_l = atol(two_str);
    
    i++;  // skip the forward slash

    while(command[i] != '/' && i<BUFFSIZE) {    
      three_str[z] = command[i];
      i++;
      z++;
    }
    three_str[z+1] = '\0'; 
    three_l = atol(three_str);


    int inz = int(two_l);
    if(inz > 3500){ inz = 3500;} else if(inz <650){inz = 650;}  //constrain
    curr_z = inz;
    servo_x.write(map(int(one_l),0,640,140,90));
   // moveSculpt();
    
  }
} // end commandParser()
