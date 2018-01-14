
/*
 * Sculpture 6 . controls three 12v large actuators via H-bridge. potentimeter return 
 * positioning     Yea I know this code could be more object object oriented 
 * X home=900, mid=585, limit=287 ; Y home=936 ,mid=680, limit=404 ; Z home 767, mid=330 , limit=108
 * I am mapping potentimeter readings down to 0 to 200 for stability. 
 */

#define BUFFSIZE 32

#define UPBOARDONPIN A7       //reads 3.3v on UP-board to see if booted up
#define LEDSWPIN 2            // switches mosfet control for LEDs

// large actuator statics **********
#define DIRPINX    12
#define DRIVEPINX  11           // pwm drive
#define POTPINX    A0          //potentimeter return 
#define HOMEX      900        // all home and limit values are actual potentimeter readings
#define LIMITX     287

#define DIRPINY    8
#define DRIVEPINY  10           // pwm drive
#define POTPINY    A1          //potentimeter return 
#define HOMEY      936
#define LIMITY     450

#define DIRPINZ    7
#define DRIVEPINZ  6           // pwm drive
#define POTPINZ    A2          //potentimeter return 
#define HOMEZ      767
#define LIMITZ     108

// Large actuator vars *******************
int pos_x = 0;       // where we are in travel 
bool currDir_x = 1;                 // holds current direction
int targetPos_x =0;               //where we want to go
bool nowMoving_x = false;

int pos_y = 0;       // where we are in travel 
bool currDir_y = 1;                 // holds current direction
int targetPos_y =0;               //where we want to go
bool nowMoving_y = false;

int pos_z = 0;       // where we are in travel 
bool currDir_z = 1;                 // holds current direction
int targetPos_z =0;               //where we want to go
bool nowMoving_z = false;

char inString[BUFFSIZE];          // Serial command buffer
bool serialEcho = true;          // do, or do not, repeat after me 


int curr_x, curr_y, curr_z;       // these hold the value 0 to 200 
int la_limit = 200;               //large limit 

bool beat = false;                //debug LED to know we are still looping 

void setup() {

  Serial.begin(115200);  //connect to serial
  
  // large actuator setup ***********************
  pinMode(DIRPINX, OUTPUT); pinMode(DRIVEPINX, OUTPUT);
  pinMode(DIRPINY, OUTPUT); pinMode(DRIVEPINY, OUTPUT);
  pinMode(DIRPINZ, OUTPUT); pinMode(DRIVEPINZ, OUTPUT);
  
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
  // run x value
   int getNewSpeed = calc_speed(1,curr_x);
   if(getNewSpeed > 0){moveMotorTo(1,curr_x,getNewSpeed);}
   
   // run y value
    getNewSpeed = calc_speed(2, curr_y);
   if(getNewSpeed > 0){moveMotorTo(2,curr_y,getNewSpeed);}
   
  // run Z value 
   getNewSpeed = calc_speed_z(3, curr_z);
   if(getNewSpeed > 0){moveMotorTo(3,curr_z,getNewSpeed);} 
}

// large actuator motor functions ***************************************************
// encoder function 

void encChk(){                // just reads the pot for position . . . so much better
  int val =0;

  if(nowMoving_x){
    val = analogRead(POTPINX);
    if(val > HOMEX){val = HOMEX;}
    else if(val < LIMITX){val = LIMITX;}
    pos_x = map(val,HOMEX,LIMITX,la_limit,0);
  }
  if(nowMoving_y){
    val = analogRead(POTPINY);
    if(val > HOMEY){val = HOMEY;}
    else if(val < LIMITY){val = LIMITY;}
    pos_y = map(val,HOMEY,LIMITY,la_limit,0);
  }
  if(nowMoving_z){
    val = analogRead(POTPINZ);
    if(val > HOMEZ){val = HOMEZ;}
    else if(val < LIMITZ){val = LIMITZ;}
    pos_z = map(val,HOMEZ,LIMITZ,la_limit,0);
  }
}

void goHome(){                // find the home position and call it 0

  pos_x = targetPos_x = map(analogRead(POTPINX), HOMEX+5, LIMITX-5,la_limit,0);
  pos_y = targetPos_y = map(analogRead(POTPINY), HOMEY+5, LIMITY-5,la_limit,0);
  pos_z = targetPos_z = map(analogRead(POTPINZ), HOMEZ+5, LIMITZ-5,la_limit,0);
  
  /*
  nowMoving = true;
  int oldPos = pos;
  moveMotor(1,585,240);
  while(analogRead(POTPIN) > 20){ ;;}
 
  pos =0; 
  nowMoving = false;
 Serial.println(analogRead(POTPIN));
 */
}

void moveMotor(int mot, bool dir,int mSpeed){       //dir 1= out, 0 = in, speed 0-254; mot 1=x,2=y,3=z
  if(mot == 1){           // x
    currDir_x = dir;
    digitalWrite(DIRPINX, dir);
    analogWrite(DRIVEPINX, mSpeed);
  }
  else if(mot == 2){      // y
    currDir_y = dir;
    digitalWrite(DIRPINY, dir);
    analogWrite(DRIVEPINY, mSpeed);
  }
  else if(mot == 3){      //z
    currDir_z = dir;
    digitalWrite(DIRPINZ, dir);
    analogWrite(DRIVEPINZ, mSpeed);
  }
}

void moveMotorTo(int mot, int targPos, int mSpeed){     // mot 1=x,2=y,3=z
  if(mot == 1){           //x
    if(targPos == pos_x){return;}
    else if(targPos > pos_x){nowMoving_x = true; targetPos_x = targPos; moveMotor(mot,0 ,mSpeed);}
    else if(targPos < pos_x){nowMoving_x = true; targetPos_x = targPos; moveMotor(mot,1 ,mSpeed);}
  }
  if(mot == 2){           //y
    if(targPos == pos_y){return;}
    else if(targPos > pos_y){nowMoving_y = true; targetPos_y = targPos; moveMotor(mot,0 ,mSpeed);}
    else if(targPos < pos_y){nowMoving_y = true; targetPos_y = targPos; moveMotor(mot,1 ,mSpeed);}
  }
  if(mot == 3){           //z
    if(targPos == pos_z){return;}
    else if(targPos > pos_z){nowMoving_z = true; targetPos_z = targPos; moveMotor(mot,0 ,mSpeed);}
    else if(targPos < pos_z){nowMoving_z = true; targetPos_z = targPos; moveMotor(mot,1 ,mSpeed);}
  }
}

void chkTarget(){                       // check to see if at target then stop
  if(nowMoving_x == true){
    if(pos_x == targetPos_x){haltMotor(1); nowMoving_x = false;}
    
  }
  if(nowMoving_y == true){if(pos_y == targetPos_y){haltMotor(2); nowMoving_y = false;}}
  if(nowMoving_z == true){if(pos_z == targetPos_z){haltMotor(3); nowMoving_z = false;}}
}

int calc_speed(int mot, int val){                // change speed by how far we need to go
  int diff=0, pos =0;
  if(mot == 1){pos = pos_x;}
  else if(mot == 2){pos = pos_y;}
  else if(mot == 3){pos = pos_z;}
  
  if(val == pos){return -1;}
  else if(val > pos){diff = val-pos;}
  else{diff = pos-val;}
  if(diff < 10){return -1;}
  else if(diff > 100){return 100;}
  else { return map(diff,10,100,40,100);}
  //else {return speed_diff[diff];}
}

int calc_speed_z(int mot, int val){                // change speed by how far we need to go
  int diff=0, pos =0;
  if(mot == 1){pos = pos_x;}
  else if(mot == 2){pos = pos_y;}
  else if(mot == 3){pos = pos_z;}
  
  if(val == pos){return -1;}
  else if(val > pos){diff = val-pos;}
  else{diff = pos-val;}
  if(diff < 10){return -1;}
  else if(diff > 100){return 230;}
  else { return map(diff,10,100,40,229);}
  //else {return speed_diff[diff];}
}


void haltMotor(int mot){
  if(mot == 1){ digitalWrite(DRIVEPINX, 0);}
  else if(mot == 2){ digitalWrite(DRIVEPINY, 0);}
  else if(mot == 3){ digitalWrite(DRIVEPINZ, 0);}
}

void printPos(){
  Serial.print("pos x  ");Serial.println(pos_x);
  Serial.print("Pot read x ");Serial.println(analogRead(POTPINX));
  Serial.print("pos y  ");Serial.println(pos_y);
  Serial.print("Pot read y ");Serial.println(analogRead(POTPINY));
  Serial.print("pos z  ");Serial.println(pos_z);
  Serial.print("Pot read z ");Serial.println(analogRead(POTPINZ));
  Serial.print("nowmoving x"); Serial.println(nowMoving_x);
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
  else if(command[i] == 's'){haltMotor(1);haltMotor(2);haltMotor(3);}
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

   
    curr_x = map(int(one_l),640,0,la_limit,0);
    curr_y = map(int(three_l),0,480,0,la_limit);

    int inz = int(two_l);
    if(inz > 3500){ inz = 3500;}
    else if(inz <650){inz = 650;}
    curr_z = map(inz,650,3500,la_limit,0);
    
    
    moveSculpt();
    
  }
} // end commandParser()
