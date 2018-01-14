void setup() {
  Serial.begin(115200);
  int la_limit = 287; 
  int la_home = 900;
  int hy = 20, ly = -20;
  Serial.print("{");
for(int i=287;i<900;i++){
  Serial.print(map(i,287,900,0,200)); Serial.print(", ");
}
Serial.print("};");
}

void loop() {
  // put your main code here, to run repeatedly:

}
