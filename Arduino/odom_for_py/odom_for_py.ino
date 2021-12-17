
#include "Wire.h"
#include <MPU6050_light.h>
MPU6050 mpu(Wire);

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle_<ArduinoHardware, 5, 5, 256, 256> nh;
////Переключение на другой сериал
//class NewHardware : public ArduinoHardware
//{
//  public:
//  NewHardware():ArduinoHardware(&Serial1, 57600){};
//};
//ros::NodeHandle_<NewHardware, 5, 5, 256, 256>  nh;




//#######################EncoderL#####################
#define Lpin_CLK 3 // Энкодер пин A
#define Lpin_DT  49 // Энкодер пин B
volatile long LPosition = 0;
long LoldPosition = 0;
float Ldist = 0;
void LEncoderRotate() {
  if (digitalRead(Lpin_CLK) == digitalRead(Lpin_DT)) {
    LPosition--;
  } else {
    LPosition++;
  }
}
//#######################EncoderL################





//#######################EncoderR#####################
#define pin_CLK 19 // Энкодер пин A
#define pin_DT  38 // Энкодер пин B
volatile long RPosition = 0;
long RoldPosition = 0;
float Rdist = 0;
void EncoderRotate() {
  if (digitalRead(pin_CLK) == digitalRead(pin_DT)) {
    RPosition++;
  } else {
    RPosition--;
  }
}
//#######################EncoderR################




//####################Motors##################




#define IN11 42
#define IN21 43
#define PWM1 9

#define IN12 37
#define IN22 36
#define PWM2 8



void rotateL() {
  digitalWrite(IN11, 1);
  digitalWrite(IN21, 0);
  digitalWrite(IN12, 0);
  digitalWrite(IN22, 1);

}




void rotateR() {
  digitalWrite(IN11, 0);
  digitalWrite(IN21, 1);
  digitalWrite(IN12, 1);
  digitalWrite(IN22, 0);

}

void run() {
  digitalWrite(IN11, 1);
  digitalWrite(IN21, 0);
  digitalWrite(IN12, 1);
  digitalWrite(IN22, 0);

}

void stop() {
  digitalWrite(IN11, 1);
  digitalWrite(IN21, 1);
  digitalWrite(IN12, 1);
  digitalWrite(IN22, 1);
}

void back() {
  digitalWrite(IN11, 0);
  digitalWrite(IN21, 1);
  digitalWrite(IN12, 0);
  digitalWrite(IN22, 1);
}



void messageCb(const geometry_msgs::Twist& message)
{
  geometry_msgs::Vector3 linear = message.linear;
  geometry_msgs::Vector3 angular = message.angular;
  float forward_vel = float(linear.x);
  float angular_vel = float(angular.z);


  //  if (forward_vel == 0 && angular_vel == 0) {
  //    stop();
  //    return;
  //  }
  //  if (forward_vel < 0 && angular_vel == 0) {
  //    regulatorR.setpoint = abs(forward_vel);
  //    regulatorL.setpoint = abs(forward_vel);
  //    back();
  //    return;
  //  }
  //  if (forward_vel > 0 && angular_vel == 0) {
  //    regulatorR.setpoint = abs(forward_vel);
  //    regulatorL.setpoint = abs(forward_vel);
  //    run();
  //    return;
  //  }
  //
  //  if (forward_vel == 0 && angular_vel > 0) {
  //    regulatorR.setpoint = abs(angular_vel);
  //    regulatorL.setpoint = abs(angular_vel);
  //    rotateR();
  //    return;
  //  }
  //  if (forward_vel == 0 && angular_vel < 0) {
  //    regulatorR.setpoint = abs(angular_vel);
  //    regulatorL.setpoint = abs(angular_vel);
  //    rotateL();
  //    return;
  //  }


}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb);


//####################Motors##################



std_msgs::Int32 R_Int_msg;
std_msgs::Int32 L_Int_msg;
std_msgs::Float32 IMU_Int_msg;
ros::Publisher r_wheel_node("r_wheel_node", &R_Int_msg);
ros::Publisher l_wheel_node("l_wheel_node", &L_Int_msg);
ros::Publisher imu_z("imu_z_node", &IMU_Int_msg);



void setup() {
  // put your setup code here, to run once:

  Wire.begin();
  mpu.begin();
  nh.initNode();
  nh.advertise(r_wheel_node);
  nh.advertise(l_wheel_node);
  nh.advertise(imu_z);
  nh.subscribe(sub);
  mpu.calcOffsets();

  pinMode(Lpin_CLK, INPUT);
  pinMode(Lpin_DT, INPUT);
  attachInterrupt(digitalPinToInterrupt(Lpin_CLK), LEncoderRotate, RISING);


  pinMode(pin_CLK, INPUT);
  pinMode(pin_DT, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_CLK), EncoderRotate, RISING);


  pinMode(IN11, OUTPUT);
  pinMode(IN21, OUTPUT);
  pinMode(PWM1, OUTPUT);


  pinMode(IN12, OUTPUT);
  pinMode(IN22, OUTPUT);
  pinMode(PWM2, OUTPUT);

}



uint32_t timer;

void loop() {
  // put your main code here, to run repeatedly:
  if (millis() - timer > 50) {
    timer = millis();
    mpu.update();
    R_Int_msg.data = RPosition;
    L_Int_msg.data = LPosition;
    IMU_Int_msg.data = mpu.getAngleZ();
    
    r_wheel_node.publish(&R_Int_msg);
    l_wheel_node.publish(&L_Int_msg);
    imu_z.publish(&IMU_Int_msg);
    
    nh.spinOnce();
  }
}
