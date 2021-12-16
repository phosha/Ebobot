/*
   rosserial Planar Odometry Example
*/



#include "GyverPID.h"
GyverPID regulatorR(100, 500, 0.1, 50);
GyverPID regulatorL(100, 500, 0.1, 50);



#include "Wire.h"
#include <MPU6050_light.h>
MPU6050 mpu(Wire);


#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
ros::NodeHandle  nh;

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
  float forward_vel = float(linear.x);
  regulatorR.setpoint = abs(forward_vel);
  regulatorL.setpoint = abs(forward_vel);
  if (forward_vel == 0) {
    stop();
    return;
  }
  if (forward_vel < 0) {

    back();
    return;
  }
  else {
    
    run();
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb);


//####################Motors##################

//##################IMU#############
int getZz() {
  //0-180 -180-0 градусов
  int b;
  if ((int)mpu.getAngleZ() % 360 < -180)b = 360 + ((int)mpu.getAngleZ() % 360);
  else if ((int)mpu.getAngleZ() % 360 > 180)b = 360 - ((int)mpu.getAngleZ() % 360);
  else b = (int)mpu.getAngleZ() % 360;
  return b;
}
//##################IMU#############

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

double x;
double y;
float theta;

char base_link[] = "/base_link";
char odom[] = "/odom";

void setup()
{

  regulatorR.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulatorR.setLimits(30, 255);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255



  regulatorL.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulatorL.setLimits(30, 255);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255


  Wire.begin();
  mpu.begin();
  // mpu.calcOffsets();
  nh.initNode();
  broadcaster.init(nh);
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



float base_angl;
uint32_t timer;
float dist, xt, yt, vr, vl;
float tmr, stp;
uint32_t old_tmr;
float Rddist, Rold_dist;
float Lddist, Lold_dist;
int ir, il;


void loop()
{

  if (millis() - timer > 50) {
    timer = millis();
    stp = 0.05;
    mpu.update();
    theta = (mpu.getAngleZ() * 71) / 4068.000;

    Rdist = RPosition / 1756.0976;
    Ldist = LPosition / 1756.0976;

    base_angl =  (Rdist - Ldist) / 0.19;
    dist = (Rdist + Ldist) / 2;

    xt = dist * cos(base_angl);
    yt = dist * sin(base_angl);


    Rddist = Rdist - Rold_dist;
    Rold_dist = Rdist;
    vr = Rddist / stp;

    Lddist = Ldist - Lold_dist;
    Lold_dist = Ldist;
    vl = Lddist / stp;

    regulatorR.input = abs(vr);
    ir = regulatorR.getResultTimer();
    analogWrite(PWM2, ir);

    regulatorL.input = abs(vl);
    il = regulatorL.getResultTimer();
    analogWrite(PWM1, il);

    // tf odom->base_link


    t.header.frame_id = odom;
    t.child_frame_id = base_link;

    t.transform.translation.x = xt;
    t.transform.translation.y = yt;


    t.transform.rotation = tf::createQuaternionFromYaw(base_angl);

    t.header.stamp = nh.now();

    broadcaster.sendTransform(t);
    nh.spinOnce();
  }

}
