
/*
 * Sculpture 2 . controls two 6v small actuators via pwm  small actuators go from 50 to 120 when servo.write
 */
#include <Servo.h>

#define SERVOLOW 50               // defensive state
#define SERVOHIGH 140             // relaxed state
#define BUFFSIZE 32
#define UPBOARDONPIN A7       //reads 3.3v on UP-board to see if booted up
#define LEDSWPIN 2            // switches mosfet control for LEDs
// servo drive pins **************
#define SERVOZA_PIN 11
#define SERVOZB_PIN 10

char inString[BUFFSIZE];          // Serial command buffer
bool serialEcho = false;          // do, or do not, repeat after me 

Servo servo_za, servo_zb ; 

int curr_x, curr_y, curr_z; 

bool beat = false;                //debug LED to know we are still looping 

void setup() {

  Serial.begin(115200);  //connect to serial
  // servo connect setup ***********************
  servo_za.attach(SERVOZA_PIN);
  servo_zb.attach(SERVOZB_PIN);
  
  pinMode(LEDSWPIN,OUTPUT);
  pinMode(13,OUTPUT);
  
 // goHome();
}

void loop() {
  if(serialCheck()){ commandParser(inString);}
  if(analogRead(UPBOARDONPIN) > 400){
    digitalWrite(LEDSWPIN, HIGH);}
  else{digitalWrite(LEDSWPIN, LOW);}
}

// servo action handlers ************************************************************
void moveSculpt(){
  
  int twitch =0;
  servo_zb.write(curr_z);
  /*
  if(curr_x > 320){twitch =map(curr_x,321,640,10,0);  }
  else if(curr_x < 320){twitch = map(curr_x,319,0,10,0);}
  int z_twitch = curr_z + twitch;
  if(z_twitch < SERVOLOW){z_twitch = SERVOLOW;}
  else if(z_twitch > SERVOHIGH){z_twitch = SERVOHIGH;}
  */
  servo_za.write(curr_z);
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
    if(inz > 1600){ inz = 1600;}
    else if(inz <650){inz = 650;}
    curr_z = map(inz,650,1600,SERVOLOW,SERVOHIGH);
   // servo_za.write(curr_x);
   // servo_zb.write(curr_y);
    
    moveSculpt();
    
  }
} // end commandParser()
