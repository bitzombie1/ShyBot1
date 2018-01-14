/*
 * Sculpture 1 . controls four 12v small actuators via pwm and one large actuator 
 * using the H-bridge  this file is for setting up the mapping 
 */
#include <Servo.h>


#define BUFFSIZE 32

// large actuator pins **********
#define DIRPIN    8
#define DRIVEPIN  3           // pwm drive
#define ENCPIN    2           // opto encoder output
// servo drive pins **************
#define SERVOXR_PIN 11 //10
#define SERVOXL_PIN 10 //11
#define SERVOYR_PIN 9
#define SERVOYL_PIN 6


// Large actuator vars *******************
int pos = 0;       // where we are in travel 
bool lastEncState;
bool currDir = 1;                 // holds current direction
int targetPos =0;               //where we want to go
bool nowMoving = false;


char inString[BUFFSIZE];          // Serial command buffer
bool serialEcho = true;

Servo servo_xr, servo_xl, servo_yr, servo_yl ; 

int curr_x, curr_y, curr_z; 
int la_limit = 305; 
bool beat = false;

void setup() {
 
  
  Serial.begin(115200);  //connect to serial
  // servo connect setup ***********************
  servo_xr.attach(SERVOXR_PIN);
  servo_xl.attach(SERVOXL_PIN);
  servo_yr.attach(SERVOYR_PIN);
  servo_yl.attach(SERVOYL_PIN);
  
  // large actuator setup ***********************
  pinMode(DIRPIN, OUTPUT); pinMode(DRIVEPIN, OUTPUT);
  pinMode(ENCPIN, INPUT_PULLUP);

  pinMode(13,OUTPUT);
  
  goHome();
}

void loop() {
  if(serialCheck()){ 
    commandParser(inString);
    }
  chkTarget();

//if(millis() % 100 == 0){if(beat == false){digitalWrite(13,HIGH); beat = true;}else{digitalWrite(13,LOW);beat = false;}} 

  encChk();
}

// servo action handlers ************************************************************
void moveSculpt(){
  /*
  int getNewSpeed = calc_speed(zMove);
   if(getNewSpeed > 0){moveMotorTo(zMove,getNewSpeed);}
   */
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
int calc_speed(int val){                // change speed by how far we need to go
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
  else if(command[i] == 'q'){printPos();}
  else if(command[i] == 's'){haltMotor();}
  else if(command[i] == 'g'){
    char one_str[11]={}; int x =0; long one_l = 0;
    char two_str[11]={}; int y =0; long two_l =0;
    char three_str[11]={};  int z =0; long three_l =0;
    char four_str[11]={};  int p =0; long four_l =0;
    char five_str[11]={}; int q=0; long five_l =0;
    
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

    i++;  // skip the forward slash
    
    while(command[i] != '/' && i<BUFFSIZE) {    
      four_str[p] = command[i];
      i++;
      p++;
    }
    four_str[p+1] = '\0'; 
    four_l = atol(four_str);

    i++;  // skip the forward slash
    
    while(command[i] != '/' && i<BUFFSIZE) {    
      five_str[q] = command[i];
      i++;
      q++;
    }
    five_str[q+1] = '\0'; 
    five_l = atol(five_str);

  int inz = int(three_l);
  if(inz > 3500){ inz = 3500;}
  int zval = map(inz,650,3500,la_limit,0);

    int getNewSpeed = calc_speed(zval);
    if(getNewSpeed > 0){moveMotorTo(zval,getNewSpeed);}

  //servo_xr.write(map(int(one_l),0,640,0,180));
//analogWrite(SERVOXL_PIN,map(int(one_l),0,640,0,255));

    
  delay(10);
  servo_xr.write(int(one_l));
  //Serial.println(int(one_l));
  //delay(10);
  servo_xl.write(int(two_l));
  //Serial.println(int(two_l));
  //delay(10);
  servo_yr.write(int(four_l));
  //delay(10);
  servo_yl.write(int(five_l));
  //delay(10);
   //int xMove = map(int(one_l),640,0,400,600);
   // int zMove = map(int(two_l),640,2400,0,300);
    //int yMove = map(int(three_l),0,480,600,400);

    
    /*
    curr_x = int(one_l); //map(int(one_l),640,0,400,600);
    curr_z = int(two_l);            //map(int(two_l),640,2400,0,300);
    curr_y = int(three_l); //map(int(three_l),0,480,600,400);
*/
  
    
   // dxMove(1,xMove);
   // dxMove(2,yMove);
  // int getNewSpeed = calc_speed(zMove);
  // if(getNewSpeed > 0){moveMotorTo(zMove,getNewSpeed);}
   
  }
} // end commandParser()
