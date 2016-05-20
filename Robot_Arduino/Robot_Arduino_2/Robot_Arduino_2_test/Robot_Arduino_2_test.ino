 #include <ros.h>

#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Int16MultiArray.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>

#define midvalue 187
#define range 30

#define FRONTRIGHT 0
#define EXTENSION 1
#define FRONTLEFT 2
#define MINING 3
#define SCOOP 4
#define BACKRIGHT 5
#define ROTATION 6
#define BACKLEFT 7

// LINEARMAXLIM : {pin: 29, index: 0}
// LINEARMINLIM : {pin: 40, index: 1}
// ROTMAXLIM : {pin: 23, index: 2}
// ROTMINLIM : {pin: 51, index: 3}

ros::NodeHandle nh;

unsigned long count;

unsigned int oldEncoderValues[8];
unsigned long oldTime;
std_msgs::UInt16MultiArray encPos;
std_msgs::Float32MultiArray encSpeed;
std_msgs::UInt16MultiArray limStat;


unsigned short motorpins[8] = {2,3,10,5,6,7,8,9}; //Set up the pins for each motor and encoder
unsigned short encoderPins[8] = {A0, A2, A3, A4, A5, A6, A7, A1};
unsigned short limitPins[4] = {29,40,23,51};


void motor_cb(const std_msgs::Int8MultiArray& message){
  if(message.data_length < 8) return;
  unsigned short value;
  
  for(int i = 0; i < 8; i++){
    if(message.data[i] != -128){
      //If we weren't sent -128, map the values sent to the motor output values
      value = (short unsigned int) map(message.data[i], -128, 127, midvalue - range, midvalue + range);
      analogWrite(motorpins[i], value);
    }
  }
}

ros::Subscriber<std_msgs::Int8MultiArray> sub("raw_motor", motor_cb);
ros::Publisher encoderPos("encoder_pos", &encPos);
ros::Publisher encoderSpeed("encoder_speed", &encSpeed);
ros::Publisher limiterStat("limiter_status", &limStat);



void setup() {
  //Initialize the outputs
  for(int i = 0; i< 8; i++){
    analogWrite(motorpins[i], 0);
  }
  
  for (int i = 0; i < 4; ++i){
     pinMode(limitPins[i],INPUT);
  }
  
  //Setup the node
  nh.initNode();
  
  nh.advertise(encoderPos);
  nh.advertise(encoderSpeed);
  //nh.advertise(limiterStat);
  
  //Init Messages
  
  encPos.data_length = 8;
  encPos.data = (uint16_t *)malloc(sizeof(uint16_t)*8);
  
 
  
  encSpeed.data_length = 8;
  encSpeed.data = (float *)malloc(sizeof(float)*8);
  
  
  limStat.data_length = 4;
  limStat.data = (uint16_t *)malloc(sizeof(uint16_t)*4);
  
  
  nh.subscribe(sub);
}


//defunct, don't use
void runAllMotors(unsigned short values[8]){
  for(int i = 0; i < 8; i++){
    analogWrite(motorpins[i], values[i]);
  }
}

void loop() {
  for(int i = 0; i < 8; i++){
    //Assign the encoder positions to a straight mapping
    encPos.data[i] = map(analogRead(encoderPins[i]), 0, 522, 0, 360);
    
    //For the change in enc, we have to do some remapping because of the way angles work
    float denc = ((float)encPos.data[i] - (float)oldEncoderValues[i]);
    
    //These should work, if they break have fun
    
    if(encPos.data[i] < 90 && oldEncoderValues[i] > 270){
      denc += 360;
    }
    
    if(encPos.data[i] > 270 && oldEncoderValues[i] < 90){
      denc -= 360;
    }
    
    //Take the change in encoder position and calculate with a time step    
    encSpeed.data[i] = 1000.0 * denc / (float)(millis() - oldTime);
    oldEncoderValues[i] = encPos.data[i];
  }
  
  for (int i = 0; i < 4; ++ i){
     limStat.data[i] = digitalRead(limitPins[i]); 
  }
 
  oldTime = millis();
  //Publish the values at a slower rate so we don't swamp the jetson
  if((++count) % 10 == 0){
    encoderSpeed.publish(&encSpeed);
    limiterStat.publish(&limStat);
  }
  nh.spinOnce();  
  delay(10);
}
