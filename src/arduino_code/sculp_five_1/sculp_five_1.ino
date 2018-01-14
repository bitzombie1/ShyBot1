
/*
 * Sculpture 5 . Controls two 12v actuators and two 6v actuators. x servo 40-140, y servo 50-120
 * both za and zb are 50-130
 */
#include <Servo.h>


#define BUFFSIZE 32

#define UPBOARDONPIN A7       //reads 3.3v on UP-board to see if booted up
#define LEDSWPIN 2            // switches mosfet control for LEDs
// servo drive pins **************
#define SERVOX_PIN 11
#define SERVOY_PIN 10
#define SERVOZA_PIN 9
#define SERVOZB_PIN 6
// servo home/limits
#define X_L 40
#define X_H 140
#define Y_L 80
#define Y_H 130
#define Z_L 50
#define Z_H 130

char inString[BUFFSIZE];          // Serial command buffer
bool serialEcho = false;          // do, or do not, repeat after me 

Servo servo_x, servo_y, servo_za, servo_zb ; 

int curr_x, curr_y, curr_z; 
 

bool beat = false;                //debug LED to know we are still looping 

void setup() {

  Serial.begin(115200);  //connect to serial
  // servo connect setup ***********************
  servo_x.attach(SERVOX_PIN);
  servo_y.attach(SERVOY_PIN);
  servo_za.attach(SERVOZA_PIN);
  servo_zb.attach(SERVOZB_PIN);
  
  pinMode(LEDSWPIN,OUTPUT);
  pinMode(13,OUTPUT);
  
  //goHome();
}

void loop() {
  if(serialCheck()){ commandParser(inString);}
  if(analogRead(UPBOARDONPIN) > 400){
    digitalWrite(LEDSWPIN, HIGH);}
    else{digitalWrite(LEDSWPIN, LOW);}
  
//  chkTarget();

//  encChk();
 
}

// servo action handlers ************************************************************
void moveSculpt(){
  int z = map(curr_z,650,3500,Z_L,Z_H);
  servo_za.write(z); servo_zb.write(z);
  delay(10);
  servo_x.write(map(curr_x,0,640,X_L,X_H));
  delay(10);
  
  if(curr_y > 300){curr_y = 480;}
  servo_y.write(map(curr_y,0,480,Y_L,Y_H));
  delay(10);
  
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

    curr_x = int(one_l); //map(int(one_l),640,0,400,600);
   // curr_z = int(two_l);            //map(int(two_l),640,2400,0,300);
    curr_y = int(three_l); //map(int(three_l),0,480,600,400);

    int inz = int(two_l);
    if(inz > 3500){ inz = 3500;}
    else if(inz <650){inz = 650;}
    curr_z = inz; 
    
    
    moveSculpt();
    
  }
} // end commandParser()
