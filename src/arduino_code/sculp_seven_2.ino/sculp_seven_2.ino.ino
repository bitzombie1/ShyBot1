/*
 * Sculpture 7 . controls one 12v large actuator using the H-bridge. pot values are from 159 to 805
 */
#define BUFFSIZE 32
#define UPBOARDONPIN A7       //reads 3.3v on UP-board to see if booted up
#define LEDSWPIN 2            // switches mosfet control for LEDs

// large actuator pins **********
#define DIRPIN    8
#define DRIVEPIN  11           // pwm drive
#define POTPIN    A3          //potentimeter return 
#define HOME      156         // absolute pot value no lower
#define LIMIT     1020

// Large actuator vars *******************
int pos = 0;       // where we are in travel 
bool currDir = 1;                 // holds current direction
int targetPos =0;               //where we want to go
bool nowMoving = false;


char inString[BUFFSIZE];          // Serial command buffer
bool serialEcho = true;          // do, or do not, repeat after me 

int curr_z;
int la_limit = 305;               //large limit 
int la_home = 0;                 //large home

void setup() {
 Serial.begin(115200);  //connect to serial
  
  // large actuator setup ***********************
  pinMode(DIRPIN, OUTPUT); pinMode(DRIVEPIN, OUTPUT);
  
  pinMode(LEDSWPIN,OUTPUT);
  pinMode(13,OUTPUT);
  goHome();
}

void loop() {
  if(serialCheck()){ commandParser(inString);}
  chkTarget();

  encChk();
  if(analogRead(UPBOARDONPIN) > 400){
    digitalWrite(LEDSWPIN, HIGH);}
    else{digitalWrite(LEDSWPIN, LOW); goHome();}
}

// large actuator motor functions ***************************************************
// encoder function 

void encChk(){                // just reads the pot for position . . . so much better
  if(nowMoving){
    int val = analogRead(POTPIN);
    if(val < HOME){val = HOME;}
    else if(val > LIMIT){val = LIMIT;}
    pos = map(val,HOME,LIMIT,0,la_limit);
  }
}

void goHome(){                // find the home position and call it 0
   pos =  map(analogRead(POTPIN), HOME-10, LIMIT+10,0, la_limit);
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
  }
}

int calc_speed(int val){                // change speed by how far we need to go
  int diff=0;
  if(val == pos){return -1;}
  else if(val > pos){diff = val-pos;}
  else{diff = pos-val;}
  if(diff < 10){return -1;}
  else if(diff > 100){return 250;}
  else { return map(diff,10,100,40,249);}

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


    int inz = int(two_l);
    if(inz > 3500){ inz = 3500;}
    else if(inz <650){inz = 650;}
    curr_z = map(inz,650,3500,la_home,la_limit);
    
    // run Z value 
    int getNewSpeed = calc_speed(curr_z);
    if(getNewSpeed > 0){moveMotorTo(curr_z,getNewSpeed);}
 
    
  }
} // end commandParser()
