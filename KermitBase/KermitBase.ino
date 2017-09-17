#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>

#define rPWM 3
#define rDir 2
#define lPWM 5
#define lDir 4
#define rBrk 6
#define lBrk 7
#define rEncA 8
#define rEncB 9

ros::NodeHandle  nh;

unsigned long previousMillis = 0;
const long interval = 500;

int rMotRPM;
int lMotRPM;
volatile long rPosition  = 0;
volatile long lPosition  = 0;

double rMotSetpoint, rMotInput, rMotOutput;
double rMotKp=2, rMotKi=1, rMotKd=0.01;
PID rMotPID(&rMotInput, &rMotOutput, &rMotSetpoint, rMotKp, rMotKi, rMotKd, DIRECT);

void cmd_vel_callback( const geometry_msgs::Twist& cmd_vel){
  rMotRPM = cmd_vel.linear.x * 100 + cmd_vel.angular.z * 100;
  lMotRPM = cmd_vel.linear.x * 100 - cmd_vel.angular.z * 100;

    if(lMotRPM < 0){
      digitalWrite(lDir, HIGH);
    }
    else{
      digitalWrite(lDir, LOW);
    }


  rMotSetpoint = rMotRPM;
  if(rMotRPM==0){
    analogWrite(rPWM, 0);
    digitalWrite(rDir, LOW);
    digitalWrite(rBrk, LOW);
    detachInterrupt(rEncA);
  }
  else{
    digitalWrite(rBrk, HIGH);
    if(rMotRPM < 0){
      digitalWrite(rDir, HIGH);
    }
    else{
      digitalWrite(rDir, LOW);
    }
    rMotPID.SetMode(AUTOMATIC);
    attachInterrupt(rEncA, rEncISR, RISING);
  }

  
//  analogWrite(rPWM, abs(rMotRPM));
//  analogWrite(lPWM, abs(lMotRPM));
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_callback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  
  pinMode(rPWM, OUTPUT);
  pinMode(rDir, OUTPUT);
  pinMode(lPWM, OUTPUT);
  pinMode(lDir, OUTPUT);
  pinMode(rBrk, OUTPUT);
  pinMode(lBrk, OUTPUT);

  
  digitalWrite(lBrk, HIGH);

  analogWrite(rPWM, 0);
  analogWrite(lPWM, 0);

  rMotPID.SetOutputLimits(80,100);
}

void loop(){
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if(rMotRPM!=0){
      rMotInput = rPosition * 120 / 1024; // converts clicks per rotation in 500uS to RPM
      rPosition = 0;
      rMotPID.Compute();
      analogWrite(rPWM, rMotOutput);
    }

    char buffer[32];
    snprintf (buffer,sizeof(buffer),"S:%.0lf,I:%.0lf,O:%.0lf",rMotSetpoint,rMotInput,rMotOutput);
    nh.loginfo(buffer);
  }
  nh.spinOnce();
  delay(1);
}

void rEncISR() {
//  if(digitalRead(rEncB)){
    rPosition++;
//  }
//  else{
//    rPosition--;
//  }
}

//void lEncISR() {
//  if(digitalRead(lEncB)){
//    lPosition++;
//  }
//  else{
//    lPosition--;
//  }
//}
