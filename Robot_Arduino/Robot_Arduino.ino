#include <ros.h>

#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Int16MultiArray.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8MultiArray.h>
#define midvalue 187
#define range 20

#define FRONTRIGHT 0
#define EXTENSION 1
#define FRONTLEFT 2
#define MINING 3
#define SCOOP 4
#define BACKRIGHT 5
#define ROTATION 6
#define BACKLEFT 7

ros::NodeHandle nh;

unsigned long count;

unsigned int oldEncoderValues[8];
unsigned long oldTime;
std_msgs::UInt16MultiArray encPos;
std_msgs::Float32MultiArray encSpeed;

void motor_cb(const std_msgs::Int8MultiArray& message){
  if(message.data_length < 8) return;
  unsigned short values[8];
  
  for(int i = 0; i < 8; i++){
    values[i] = (short unsigned int) map(message.data[i], -128, 127, midvalue - range, midvalue + range);
  }
  runAllMotors(values);
}

ros::Subscriber<std_msgs::Int8MultiArray> sub("raw_motor", motor_cb);
ros::Publisher encoderPos("encoder_pos", &encPos);
ros::Publisher encoderSpeed("encoder_speed", &encSpeed);


unsigned short motorpins[8] = {2,3,10,5,6,7,8,9};
unsigned short encoderPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};


void setup() {
  for(int i = 0; i< 8; i++){
    analogWrite(motorpins[i], 0);
  }
  
  nh.initNode();
  
  nh.advertise(encoderPos);
  nh.advertise(encoderSpeed);
  
  encPos.data_length = 8;
  encPos.data = (uint16_t *)malloc(sizeof(uint16_t)*8);
  
 
  
  encSpeed.data_length = 8;
  encSpeed.data = (float *)malloc(sizeof(float)*8);
  
  
  
  nh.subscribe(sub);
}



void runAllMotors(unsigned short values[8]){
  for(int i = 0; i < 8; i++){
    analogWrite(motorpins[i], values[i]);
  }
}

void loop() {
  for(int i = 0; i < 8; i++){
    encPos.data[i] = map(analogRead(encoderPins[i]), 0, 522, 0, 360);
    float denc = ((float)encPos.data[i] - (float)oldEncoderValues[i]);
    
    if(encPos.data[i] < 90 && oldEncoderValues[i] > 270){
      denc += 360;
    }
    
    if(encPos.data[i] > 270 && oldEncoderValues[i] < 90){
      denc -= 360;
    }
    
    
    encSpeed.data[i] = 1000.0 * denc / (float)(millis() - oldTime);
    oldEncoderValues[i] = encPos.data[i];
  }
 
  oldTime = millis();
  if((++count) % 10 == 0){
    encoderPos.publish( &encPos);
    encoderSpeed.publish(&encSpeed);
  }
  nh.spinOnce();  
  delay(10);
}
