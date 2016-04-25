#define midvalue 187

#define range 20
#define dataBufferPin 42
#define LED1 30
#define LED2 31
#define LED3 32

  // 1 - Front Right
  // 2 - Extension
  // 3 - Front Left
  // 4 - Mining
  // 5 - Scoop
  // 6 - Back Right
  // 7 - Arm Rotation
  // 8 - Back Left

#define FRONTRIGHT 0
#define EXTENSION 1
#define FRONTLEFT 2
#define MINING 3
#define SCOOP 4
#define BACKRIGHT 5
#define ROTATION 6
#define BACKLEFT 7

#define CLK 38
#define DAT 39
#define PUL 40

int B = 0x01;
int Y = 0x02;
int select = 0x04;
int start = 0x08;
int up = 0x10;
int down  =  0x20;
int left  =  0x40;
int right = 0x80;
int A = 0x100;
int X = 0x200;
int L = 0x400;
int R = 0x800;

unsigned short motorpins[8] = {2,3,10,5,6,7,8,9};
unsigned short currentMotor = 0;
boolean olddataBuffer = true;

unsigned short oldvalues[8] = {midvalue,midvalue,midvalue,midvalue,midvalue,midvalue,midvalue,midvalue};


void setup() {
  for(int i = 0; i< 8; i++){
    analogWrite(motorpins[i], 0);
  }
  pinMode(CLK, OUTPUT);
  pinMode(DAT, INPUT);
  pinMode(PUL, OUTPUT);
  Serial.begin(9600);
  
}

void runAllMotors(unsigned short values[8]){
  for(int i = 0; i < 8; i++){/*
    int tempval = oldvalues[i];
    tempval += constrain((int)values[i] - (int)oldvalues[i], -5, 5);
    oldvalues[i] = tempval;*/
    analogWrite(motorpins[i], values[i]);
  }
}

void loop() {
  unsigned short motorValues[8] = {0,0,0,0,0,0,0,0};
  uint16_t dataBuffer = 0;
  /*
  //When the dataBuffer is pressed, move to the next motor
  if(olddataBuffer != digitalRead(dataBufferPin) && digitalRead(dataBufferPin) == HIGH){
    currentMotor++;
    if(currentMotor >= sizeof(motorpins)){
      currentMotor = 0;
    }
    delay(100);
  }
  */

  digitalWrite(PUL, HIGH);
  delayMicroseconds(16);
  digitalWrite(PUL, LOW);
  delayMicroseconds(16);
  
  for(int i = 0; i < 16; i++){
    dataBuffer |= ((digitalRead(DAT) == HIGH)? 1:0) << i;
    digitalWrite(CLK, HIGH);
    delayMicroseconds(16);
    digitalWrite(CLK, LOW);
    delayMicroseconds(16);
  }
  dataBuffer = ~dataBuffer;
  

  /*
  for(int i = 0; i < 8; i++){
    motorValues[i] = 0;
  }
  
  motorValues[currentMotor] =map(analogRead(A0), 0, 1023, midvalue - range, midvalue + range);*/

  int goup, godown, goleft, goright;
  goup = (up & dataBuffer) ? 1 : 0;
  godown = (down & dataBuffer) ? 1 : 0;
  goleft = (left & dataBuffer) ? 1 : 0;
  goright = (right & dataBuffer) ? 1 : 0;

  int leftVal = (goup || goright) ? 1 : ((goleft || godown)? -1 : 0);
  int rightVal = (goup || goleft) ? 1 : ((goright || godown)? -1 : 0);

  int INVFL, INVFR, INVBL, INVBR;
  //Set to -1 to invert
  INVFL = -1;
  INVFR = 1;
  INVBL = -1;
  INVBR = 1;

  if(leftVal < 0){
    motorValues[FRONTLEFT] = midvalue - (range * INVFL);
    motorValues[BACKLEFT] = midvalue - (range * INVBL);
  } else if (leftVal > 0){
    motorValues[FRONTLEFT] = midvalue + (range * INVFL);
    motorValues[BACKLEFT] = midvalue + (range * INVBL);
  } else{
    motorValues[FRONTLEFT] = midvalue;
    motorValues[BACKLEFT] = midvalue;
  }

  if(rightVal < 0){
    motorValues[FRONTRIGHT] = midvalue - (range * INVFR);
    motorValues[BACKRIGHT] = midvalue - (range * INVBR);
  } else if (rightVal > 0){
    motorValues[FRONTRIGHT] = midvalue + (range * INVFR);
    motorValues[BACKRIGHT] = midvalue + (range * INVBR);
  } else{
    motorValues[FRONTRIGHT] = midvalue;
    motorValues[BACKRIGHT] = midvalue;
  }

  if( L & dataBuffer ){
    motorValues[SCOOP] = midvalue + range;
  }else if (R & dataBuffer){
    motorValues[SCOOP] = midvalue - range;
  } else{
    motorValues[SCOOP] = midvalue;
  }

  if( A & dataBuffer ){
    motorValues[EXTENSION] = midvalue + range;
  }else if (B & dataBuffer){
    motorValues[EXTENSION] = midvalue - range;
  } else{
    motorValues[EXTENSION] = midvalue;
  }

  if( X & dataBuffer ){
    motorValues[ROTATION] = midvalue + range;
  }else if (Y & dataBuffer){
    motorValues[ROTATION] = midvalue - range;
  } else{
    motorValues[ROTATION] = midvalue;
  }

  if( start & dataBuffer ){
    motorValues[MINING] = midvalue + range;
  }else if (select & dataBuffer){
    motorValues[MINING] = midvalue - range;
  } else{
    motorValues[MINING] = midvalue;
  }
  
  
  
  runAllMotors(motorValues);

  for(int i = 0; i < 8; i++){
    Serial.print(motorValues[i]);
    Serial.print(",");
  }
  Serial.println();

  delay(50);
  

}
