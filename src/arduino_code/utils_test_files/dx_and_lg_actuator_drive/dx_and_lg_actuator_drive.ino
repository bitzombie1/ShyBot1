



#define AX_START                    255
#define AX_ID                       3
#define AX_ID_LENGTH                4
#define AX_TORQUE_LENGTH            4
#define AX_WRITE_DATA               3
#define AX_TORQUE_ENABLE            24
#define AX_LED_LENGTH               4
#define AX_LED                      25
#define TX_DELAY_TIME               800 //400
#define AX_GOAL_LENGTH              5
#define AX_GOAL_POSITION_L          30
#define AX_BD_LENGTH                4
#define AX_BAUD_RATE                4

#define sendData(args)  (Serial.write(args))    // Write Over Serial
#define delayus(args) (delayMicroseconds(args))  // Delay Microseconds

// large actuator pins **********
#define DIRPIN    8
#define DRIVEPIN  3           // pwm drive
#define CSPIN     14          //current sense pin
#define ENCPIN    2           // opto encoder output

unsigned char Checksum;

// Large actuator vars *******************
int pos = 0;       // where we are in travel 
bool lastEncState;
bool currDir = 1;                 // holds current direction
int targetPos =0;               //where we want to go
bool nowMoving = false;


char inString[32];          // Serial command buffer
bool serialEcho = false;

void setup() {
  // dx serial and regular serial setup *****************
  Serial.begin(1000000);  // 115200 Dynamixel.begin(1000000,2);  // Inicialize the servo at 1Mbps 
  //while (!Serial1) ;
  delay(1000);
  dxSetBD(2,115200);  //lower the baud rate on the motor
  dxSetBD(1,115200);
  Serial.end();
  Serial.begin(115200);
  delay(500); 
  Serial.begin(115200);  //reconnect to serial
  while (!Serial) ;
  //Serial.println("Setup done  ");
  


  // large actuator setup ***********************
  pinMode(DIRPIN, OUTPUT); pinMode(DRIVEPIN, OUTPUT);
  pinMode(ENCPIN, INPUT_PULLUP);
  goHome();
}

void loop() {
  serialCheck();
  chkTarget();

  encChk();
}

// dx servo motor functions ********************************
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

// large actuator motor functions ***************************************************
// encoder function 
void encChk(){
  if(nowMoving){
      bool now_p = digitalRead(ENCPIN);  //bool now_p = bitRead(PIND,2);
      if(now_p == 0 && lastEncState == 1){
          if(currDir == 0){pos--;lastEncState = 0;} else{pos++; lastEncState = 0;}
      }
      else if(now_p == 1 && lastEncState == 0){lastEncState = 1;}
  }
}

void goHome(){
  moveMotor(0,200);
  delay(10000);
  pos =0; 
  printPos();
 
}
void moveMotor(bool dir,int mSpeed){       //dir 1= out, 0 = in, speed 0-254
  currDir = dir;
  digitalWrite(DIRPIN,dir); // set pin 8 //digitalWrite(DIRPIN,dir);
  analogWrite(DRIVEPIN, mSpeed);
}
void moveMotorTo(int targPos, int mSpeed){
  if(targPos == pos){return;}
  else if(targPos > pos){nowMoving = true; targetPos = targPos; moveMotor(1, mSpeed);}
  else if(targPos < pos){nowMoving = true; targetPos = targPos; moveMotor(0, mSpeed);}
}
void chkTarget(){                       // check to see if at target then stop
  if(nowMoving == true){
    if(pos == targetPos){ haltMotor(); nowMoving = false;}
    //else{adjustSpeed();}
  }
}
int calc_speed(int val){
  int diff=0;
  if(val == pos){return -1;}
  else if(val > pos){diff = val-pos;}
  else{diff = pos-val;}
  if(diff < 10){return -1;}
  else if(diff > 100){return 240;}
  else { return map(diff,10,100,40,239);}
}

int disToGo(){                      // calculate how far to target and adjust speed
  return targetPos - pos;
}

void haltMotor(){
  digitalWrite(DRIVEPIN, 0);   // set pin 3 low //digitalWrite(DRIVEPIN, 0); 
}
void printPos(){
  Serial.println(pos);
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
    if (serialEcho) {
      Serial.println(inString);
    }
    commandParser(inString);
    return true;
  }
  return false;
}

void commandParser(char * command){
  int i =0;
  if(command[i] == 'h'){goHome();} // home command
  else if(command[i] == 'q'){printPos();}
  else if(command[i] == 's'){haltMotor();}
  else if(command[i] == 'g'){
    char one_str[11]={}; int x =0; long one_l = 0;
    char two_str[11]={}; int y =0; long two_l =0;
    char three_str[11]={};  int z =0; long three_l =0;
    while(command[i] != '/' && i<32) {
      i++;
      one_str[x] = command[i];
      x++;
    }
    one_str[x+1] = '\0'; 
    one_l = atol(one_str);
       
    i++;  // skip the forward slash
    
    while(command[i] != '/' && i<32) {    
      two_str[y] = command[i];
      i++;
      y++;
    }
    two_str[y+1] = '\0'; 
    two_l = atol(two_str);
    
    i++;  // skip the forward slash

    while(command[i] != '/' && i<32) {    
      three_str[z] = command[i];
      i++;
      z++;
    }
    three_str[z+1] = '\0'; 
    three_l = atol(three_str);

   int xMove = map(int(one_l),640,0,400,600);
    int zMove = map(int(two_l),640,2400,0,300);
    int yMove = map(int(three_l),0,480,600,400);
    
   // dxMove(1,xMove);
   // dxMove(2,yMove);
   int getNewSpeed = calc_speed(zMove);
   if(getNewSpeed > 0){moveMotorTo(zMove,getNewSpeed);}
   /*
   if(zMove < (pos - 100) || zMove > (100 + pos)){moveMotorTo(zMove,240); }
   else if (zMove < (pos - 50) || zMove > (50 + pos)){moveMotorTo(zMove,140); }
   else if (zMove < (pos - 20) || zMove > (20 + pos)){moveMotorTo(zMove,80); }
   else if (zMove < (pos - 10) || zMove > (10 + pos)){moveMotorTo(zMove,40); }
   */
    //Serial.print(1);
  }
} // end commandParser()
