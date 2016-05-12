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
  
  encPos.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  encPos.layout.dim_length = 1;
  encPos.layout.dim[0].label = "Label";
  encPos.layout.dim[0].size = 8;
  encPos.layout.dim[0].stride = 1*8;
  encPos.layout.data_offset = 0;
  encPos.data = (uint16_t *)malloc(sizeof(uint16_t)*8);
  
  encSpeed.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  encSpeed.layout.dim_length = 1;
  encSpeed.layout.dim[0].label = "Label";
  encSpeed.layout.dim[0].size = 8;
  encSpeed.layout.dim[0].stride = 1*8;
  encSpeed.layout.data_offset = 0;
  encSpeed.data = (float *)malloc(sizeof(float)*8);
  
  //nh.advertise(encoderSpeed);
  //nh.subscribe(sub);
}



void runAllMotors(unsigned short values[8]){
  for(int i = 0; i < 8; i++){
    analogWrite(motorpins[i], values[i]);
  }
}

void loop() {
  for(int i = 0; i < 8; i++){
    //encPos.data[i] = map(analogRead(encoderPins[i]), 0, 1024, 0, 360);
    //encSpeed.data[i] = 1000 * (float)(analogRead(encoderPins[i]) - oldEncoderValues[i]) / (millis() - oldTime);
    oldEncoderValues[i] = analogRead(encoderPins[i]);
  }
  encPos.data_length = 8;
  encSpeed.data_length = 8;
  oldTime = millis();
  
  encoderPos.publish( &encPos);
  //encoderSpeed.publish(&encSpeed);
  nh.spinOnce();  
  delay(100);
}
