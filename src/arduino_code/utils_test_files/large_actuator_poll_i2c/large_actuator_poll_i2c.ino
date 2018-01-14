
#include <avr/io.h>
#include <Wire.h>

#define DIRPIN    8
#define DRIVEPIN  3           // pwm drive
#define CSPIN     14          //current sense pin
#define ENCPIN    2           // opto encoder output

#define BUADRATE 115200

char inString[32];              // Serial command buffer
bool mesgFlag = false;

int pos = 0;       // where we are in travel 
bool lastEncState;
bool currDir = 1;                 // holds current direction
int targetPos =0;               //where we want to go
bool nowMoving = false;


void setup() {
  pinMode(DIRPIN, OUTPUT); pinMode(DRIVEPIN, OUTPUT);
  pinMode(ENCPIN, INPUT_PULLUP);
 
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
  Serial.begin(BUADRATE);
  goHome();

  Serial.println("setup done");
}

void loop() {
 
  if(mesgFlag){commandParser(inString); }
  chkTarget();
  encChk();

}
// encoder function **************************************************
void encChk(){
  if(nowMoving){
      bool now_p = bitRead(PIND,2);
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
}

void moveMotor(bool dir,int mSpeed){       //dir 1= out, 0 = in, speed 0-254
  currDir = dir;
  bitWrite(PORTB,0,dir);  // set pin 8 //digitalWrite(DIRPIN,dir);
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

int disToGo(){                      // calculate how far to target and adjust speed
  return targetPos - pos;
}

void haltMotor(){
  bitWrite(PORTD,2,0);  // set pin 3 low //digitalWrite(DRIVEPIN, 0); 
}
// Serial functions **************************************************
void printPos(){
  Serial.println(pos);
}

// I2C serial functions **********************************************
void receiveEvent(int howMany){         // inturupt function called when master sends message
  int i=0;
  while(1 < Wire.available()){ inString[i++] = Wire.read(); }   // recieve byte as a char
  inString[i++]='\0';
  mesgFlag = true;
  Serial.println("mess");
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
    Serial.println(command);
  }
  mesgFlag = false;
} // end commandParser()
