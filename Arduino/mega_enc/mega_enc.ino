
#include "GyverPID.h"
GyverPID regulatorR(100, 500, 0.1, 20);
GyverPID regulatorL(100, 500, 0.1, 20);


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
long oldPosition = 0;
float Rdist = 0;
void EncoderRotate() {
  if (digitalRead(pin_CLK) == digitalRead(pin_DT)) {
    RPosition++;
  } else {
    RPosition--;
  }
}
//#######################EncoderR################





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

void setup() {

  regulatorR.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulatorR.setLimits(30, 255);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
  regulatorR.setpoint = 0.1;


  regulatorL.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulatorL.setLimits(30, 255);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
  regulatorL.setpoint = 0.1;

  pinMode(Lpin_CLK, INPUT);
  pinMode(Lpin_DT, INPUT);
  attachInterrupt(digitalPinToInterrupt(Lpin_CLK), LEncoderRotate, RISING);


  pinMode(pin_CLK, INPUT);
  pinMode(pin_DT, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_CLK), EncoderRotate, RISING);
  Serial.begin (9600);




  pinMode(IN12, OUTPUT);
  pinMode(IN22, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(IN11, OUTPUT);
  pinMode(IN21, OUTPUT);
  pinMode(PWM1, OUTPUT);
  run();
}
float base_angl;
float dist, xt, yt, vr, vl;
float tmr, stp;
uint32_t old_tmr;
float Rddist, Rold_dist;
float Lddist, Lold_dist;
int ir, il;
void loop() {
  Rdist = RPosition / 1756.0976;
  Ldist = LPosition / 1756.0976;
  base_angl =  (Rdist - Ldist) / 0.18;
  dist = (Rdist + Ldist) / 2;
  xt = dist * cos(base_angl);
  yt = dist * sin(base_angl);




  stp = (float)(millis() - tmr) / 1000;
  tmr = millis();
  Rddist = Rdist - Rold_dist;
  Rold_dist = Rdist;
  vr = Rddist / stp;


  Lddist = Ldist - Lold_dist;
  Lold_dist = Ldist;
  vl = Lddist / stp;


  regulatorR.input = vr;
  ir = regulatorR.getResultTimer();
  analogWrite(PWM2, ir);

  regulatorL.input = vl;
  il = regulatorL.getResultTimer();
  analogWrite(PWM1, il);


  Serial.print(il);
  Serial.print(" ");
  Serial.print(vl * 100);
  Serial.print(" ");
  Serial.print(ir);
  Serial.print(" ");
  Serial.println(vr * 100);

}
