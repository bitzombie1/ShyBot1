
/*
 * Sculpture 3 . Controls three 6v actuators named Z12,Z8,and Z4 for their position 
 * on a clock (ie 12 o-clock etc.). Also the 12v AX-12 servo was thrown on the 6v 
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
#define SERVOZ12_PIN 11
#define SERVOZ4_PIN 10
#define SERVOZ8_PIN 6

// servo timing/state 
#define SERVOWAIT 3000         //time ms we wait for servo travel
unsigned long servo_dur_t;    // time the servo has been running
bool in_motion = false;         //are we now in motion?
int state = 5;                  // state of distance 5=max, 0=min

int state_mid[] = {888,1363,1838,2313,2788,3263};     //the mid point of each state

// servo home/limits
#define SERVOZHOME 40

char inString[BUFFSIZE];          // Serial command buffer
bool serialEcho = true;          // do, or do not, repeat after me 
unsigned char Checksum;

Servo servo_z12, servo_z4, servo_z8; 

int  curr_z = 3500;                    //our current depth value
int  targ_z =3500;
int pos_lst[] = {200,300,400,500,600,700,800};    //where our ax-12 servo can move to

bool beat = true;                //debug LED to know we are still looping 
unsigned long now_t, prev_t, delta_t;  //timers

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
  servo_z12.attach(SERVOZ12_PIN);
  servo_z4.attach(SERVOZ4_PIN);
  servo_z8.attach(SERVOZ8_PIN);

  // send the servos home on startup. important for this one.
  goHome();
  
  pinMode(LEDSWPIN,OUTPUT);
  pinMode(13,OUTPUT);
  
  
}

void loop() {
   now_t = millis();
  if(now_t < prev_t){delta_t = now_t + (4294967295 - prev_t);} //millis rollover check
  else{delta_t = now_t - prev_t;}

  if(in_motion){chkMotion();}
  else{chkDist();}
  if(serialCheck()){ commandParser(inString);}
  
  if(analogRead(UPBOARDONPIN) > 400){
    digitalWrite(LEDSWPIN, HIGH);}
    else{digitalWrite(LEDSWPIN, LOW); curr_z = 3500; goHome();}
    
 
  if((curr_z < 1200) && now_t % 2000 == 0){ dxMoveSpeed(1,pos_lst[int(random(0, 7))],60);}

  prev_t = now_t;
}

// servo action handlers ************************************************************
void goHome(){
   servo_z8.write(SERVOZHOME); 
   delay(100);
   servo_z4.write(SERVOZHOME);
   delay(100); 
   servo_z12.write(SERVOZHOME); 
  delay(8000);
  state = 5; 
  in_motion = false;
  curr_z = 3500;
  targ_z = 3500;
}
void setMotion(){               //set motion timers/flag
  servo_dur_t = 0;
  in_motion = true;
}

void chkMotion(){               // check and maybe reset motion timer/flag
  if(servo_dur_t  > SERVOWAIT){ in_motion = false;}
  else{servo_dur_t += delta_t;}
}

void chkDist(){
  
  if(curr_z == targ_z){return;}       //we are there nothing to do
  int new_state = chkState(targ_z);
//Serial.print("new_state "); Serial.println(new_state);

  if(new_state == state){             // we are close
      setMotion();
      moveSculpt();
      curr_z = targ_z;
    }
  else if(new_state > state){
      state++;
      curr_z = state_mid[state];
      setMotion();
      moveSculpt();
 //  Serial.println("state++");
    }
  else if(new_state < state){
      
      state--;
      curr_z = state_mid[state];
      setMotion();
      moveSculpt();
  //   Serial.println("state--");
    }


  
}

int chkState(int z_val){        //we seperate possible z values into states
  int state_out=0;
  if(z_val > 649 && z_val <= 1125){state_out = 0;}
  else if(z_val > 1125 && z_val <= 1600){state_out =1;}
  else if(z_val > 1600 && z_val <= 2075){state_out =2;}
  else if(z_val > 2075 && z_val <= 2550){state_out=3;}
  else if(z_val > 2550 && z_val <= 3025){state_out=4;}
  else if(z_val >3025 && z_val <= 3500){state_out=5;}
  return state_out;
}

void moveSculpt(){
  int z12, z4, z8;
 //Serial.print("state "); Serial.println(state);
 //Serial.print("targ_z "); Serial.println(targ_z);
 //Serial.print("curr_z "); Serial.println(curr_z);
  if(state==0){
    z12 = map(curr_z,650,1125,90,80); z4 = 90; z8 = 110;
  }
  else if(state==1){
    z12 = map(curr_z,1126,1600,80,60); z4 = 90; z8 = 110;
  }
  else if(state==2){
   z12 = map(curr_z,1601,2075,60,40); z4 = map(curr_z,1601,2075,90,80); z8 = 110;
  }
  else if(state==3){
    z12 = SERVOZHOME; z4 = map(curr_z,2076,2550,80,60); z8 = map(curr_z,2076,2550,110,90);
  }
  else if(state==4){
    z12 = SERVOZHOME; z4 = map(curr_z,2551,3025,60,40); z8 = map(curr_z,2551,3025,90,80);
  }
  else if(state==5){
    z12 = SERVOZHOME; z4 = SERVOZHOME; z8 = map(curr_z,3026,3500,80,40);
  }
   
  servo_z8.write(z8);
  delay(10);
  servo_z4.write(z4);
  delay(10);
  servo_z12.write(z12);
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
  if(command[i] == 'h'){goHome();} // home command
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
    
    targ_z = inz;

    /*
    if(new_state == state){
      curr_z = inz; 
      moveSculpt();
    }
    else if(new_state > state){
      if(!in_motion){
        state++;
        curr_z = state_mid[state];
        setMotion();
        moveSculpt();
      }
    }
    else if(new_state < state){
      if(!in_motion){
        state--;
        curr_z = state_mid[state];
        setMotion();
        moveSculpt();
      }
    }
    */
   // curr_z = inz; 
    /*
    servo_z8.write(int(three_l));
    delay(10);
    servo_z4.write(int(two_l));
    delay(10);
    servo_z12.write(int(one_l));
    */
   // moveSculpt();
    
  }
} // end commandParser()
