
/*
 * Sculpture 1 . 200 down ,1000 up controls four 12v small actuators via pwm and one large actuator 
 * using the H-bridge small actuators go from 40 to 140 when servo.write
 */
#include <Servo.h>


#define BUFFSIZE 32

#define UPBOARDONPIN A7       //reads 3.3v on UP-board to see if booted up
#define LEDSWPIN 2            // switches mosfet control for LEDs

#define VISITHOME 5           // how many consecutive limit commands before returning home
// large actuator pins **********
#define DIRPIN    8
#define DRIVEPIN  3           // pwm drive
#define POTPIN    A0          //potentimeter return 
// servo drive pins **************
#define SERVOXR_PIN 11
#define SERVOXL_PIN 10
#define SERVOYR_PIN 9
#define SERVOYL_PIN 6

// Large actuator vars *******************
int pos = 0;       // where we are in travel 
bool currDir = 1;                 // holds current direction
int targetPos =0;               //where we want to go
bool nowMoving = false;


char inString[BUFFSIZE];          // Serial command buffer
bool serialEcho = false;          // do, or do not, repeat after me 

Servo servo_xr, servo_xl, servo_yr, servo_yl ; 

int curr_x, curr_y, curr_z; 
int la_limit = 305;               //large limit 

bool beat = false;                //debug LED to know we are still looping 

void setup() {

  Serial.begin(115200);  //connect to serial
  // servo connect setup ***********************
  servo_xr.attach(SERVOXR_PIN);
  servo_xl.attach(SERVOXL_PIN);
  servo_yr.attach(SERVOYR_PIN);
  servo_yl.attach(SERVOYL_PIN);
  
  // large actuator setup ***********************
  pinMode(DIRPIN, OUTPUT); pinMode(DRIVEPIN, OUTPUT);
  //pinMode(ENCPIN, INPUT_PULLUP);

  pinMode(13,OUTPUT);
  pinMode(LEDSWPIN,OUTPUT);
  goHome();
}

void loop() {
  if(serialCheck()){ commandParser(inString);}
  chkTarget();

  encChk();
  if(analogRead(UPBOARDONPIN) > 400){
    digitalWrite(LEDSWPIN, HIGH);}
    else{digitalWrite(LEDSWPIN, LOW);}
}

// servo action handlers ************************************************************
void moveSculpt(){

  // run x right value
   int h=10,l= -10;
   servo_xr.write((map(curr_x,0,640,110,70) + map(curr_z,0,la_limit,l,h)));
   // run x left value
   int hr= 10, lr = -10;
   
   servo_xl.write(( map(curr_x,0,640,60,100) + map(curr_z, 0,la_limit,lr,hr)));
   // run y right value
   int hy = 20, ly = -20;
   servo_yr.write((map(curr_y,0,480,70,90) + map(curr_z,0,la_limit,hy,ly)));
   // run y left value
   
  servo_yl.write((map(curr_y,0,480,70,100) + map(curr_z,0,la_limit,hy,ly)));
  // run Z value 
  int getNewSpeed = calc_speed(curr_z);
   if(getNewSpeed > 0){moveMotorTo(curr_z,getNewSpeed);}
   
   
}

// large actuator motor functions ***************************************************
// encoder function 

void encChk(){                // just reads the pot for position . . . so much better
  if(nowMoving){
    int val = analogRead(POTPIN);
    if(val < 20){val =20;}
    else if(val > 800){val = 800;}
    pos = map(val,20,800,0,la_limit);
  }
}

void goHome(){                // find the home position and call it 0
  if(millis() % 2 == 0){servo_yr.write(50); servo_yl.write(120);}
  else {servo_yr.write(120);  servo_yl.write(50);}
  delay(50);
  nowMoving = true;
  int oldPos = pos;
  moveMotor(0,240);
  while(analogRead(POTPIN) > 20){ ;;}
 
  pos =0; 
  nowMoving = false;
 Serial.println(analogRead(POTPIN));
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
  //else {return speed_diff[diff];}
}

int disToGo(){                      // calculate how far to target and adjust speed
  return targetPos - pos;
}

void haltMotor(){
  digitalWrite(DRIVEPIN, 0);   // set pin 3 low //digitalWrite(DRIVEPIN, 0); 
}
void printPos(){
  Serial.print("pos  ");Serial.println(pos);
  Serial.println(analogRead(POTPIN));
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

   //int xMove = map(int(one_l),640,0,400,600);
   // int zMove = map(int(two_l),640,2400,0,300);
    //int yMove = map(int(three_l),0,480,600,400);
    curr_x = int(one_l); //map(int(one_l),640,0,400,600);
   // curr_z = int(two_l);            //map(int(two_l),640,2400,0,300);
    curr_y = int(three_l); //map(int(three_l),0,480,600,400);

    int inz = int(two_l);
    if(inz > 3500){ inz = 3500;}
    else if(inz <650){inz = 650;}
    curr_z = map(inz,650,3500,la_limit,0);
    
    
    moveSculpt();
    
  }
} // end commandParser()
