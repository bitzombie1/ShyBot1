
#include <avr/io.h>

#define DIRPIN    8
#define DRIVEPIN  3           // pwm drive
#define CSPIN     14          //current sense pin
#define ENCPIN    2           // opto encoder output


#define BUADRATE 115200

char inString[32];              // Serial command buffer
bool serialEcho = false;

int pos = 0;       // where we are in travel 
bool lastEncState;
bool currDir = 1;                 // holds current direction
int targetPos =0;               //where we want to go
bool nowMoving = false;

unsigned long now_t, prev_t, delta_t;  //timers 

void setup() {
  pinMode(DIRPIN, OUTPUT); pinMode(DRIVEPIN, OUTPUT);
  pinMode(ENCPIN, INPUT_PULLUP);
 
  
  Serial.begin(BUADRATE);
  goHome();
}

void loop() {
  now_t = millis();
  if(now_t < prev_t){delta_t = now_t + (4294967295 - prev_t);} //millis rollover check
  else{delta_t = now_t - prev_t;}
 
  serialCheck();
 // if(now_t % 300 == 0){ printPos();}//Serial.print(state); Serial.print(" <state  direction> ");Serial.println(currDir);}
  chkTarget();

  encChk();
  
  prev_t = now_t;
}
// encoder function **************************************************
void encChk(){
  if(nowMoving){
      bool now_p = digitalRead(ENCPIN);  //bool now_p = bitRead(PIND,2);
      if(now_p == 0 && lastEncState == 1){
          if(currDir == 0){pos--;lastEncState = 0;} else{pos++; lastEncState = 0;}
      }
      else if(now_p == 1 && lastEncState == 0){lastEncState = 1;}
  }
}

// motor functions ***************************************************
void goHome(){
  moveMotor(0,200);
  delay(10000);
  pos =0; 
  printPos();
  /*
  int prev_state = state;
  bool loopin = true;
  while(loopin){
    prev_state = state;
    delay(10);
    if(prev_state == state){state =0; loopin = false;}
  }
  */
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
int disToGo(){                      // calculate how far to target and adjust speed
  return targetPos - pos;
}


void haltMotor(){
  digitalWrite(DRIVEPIN, 0);   // set pin 3 low //digitalWrite(DRIVEPIN, 0); 
}
// Serial functions **************************************************
void printPos(){
  Serial.println(pos);
}

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
    
   // moveMotor(bool(one_l),int(two_l));
   moveMotorTo(int(one_l),int(two_l)); 
    
  }
} // end commandParser()
