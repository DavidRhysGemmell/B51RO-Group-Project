#include "PinChangeInterrupt.h"
#include <ServoInput.h>
#include <Servo.h>  
#define Channel1 8
#define Channel2 9
#define Channel3 10
#define Channel4 11
#define Channel5 12
#define Channel6 13


ServoInputPin<2> ch1; // Reciever channel 1
ServoInputPin<3> ch2;// Reciever channel 2
ServoInputPin<4> ch3;// Reciever channel 3
ServoInputPin<5> ch4;// Reciever channel 4
ServoInputPin<6> ch5;// Reciever channel 5
ServoInputPin<7> ch6;// Reciever channel 6

Servo Servo1; 
Servo Servo2; 
Servo Servo3; 
Servo Servo4; 
Servo Servo5;
Servo Servo6;  
int Servo1Angle=0;
int Servo2Angle=0;
int Servo3Angle=0;
int Servo4Angle=0;
int Servo5Angle=0;
int Servo6Angle=0;


float d_1 = 77.6; //Height to first joint
#define l_1 150 // Lenght of first arm
#define l_2 150 // Length of second arm
#define l_3 50 // Length of third arm

int x = 150; //Initalise End coordinates
int y = 50;
int z = 50;
float phi = PI; // End posture of arm, in radians. Could be editable.

float xp = 0;
float zp = 0;
double ctheta_1 = 0;
double stheta_1 = 0;
double theta_1 = 0;
double x3 = 0;
double y3 = 0;
double z3 = 0;
double x3p = 0;
double z3p = 0;
double ctheta_3 = 0;
double stheta_3 = 0;
double theta_3 = 0;
double sbeta = 0;
double cbeta = 0;
double beta = 0;
double cpsi = 0;
double spsi = 0;
double psi = 0;
double theta_2 = 0;
double theta_4 = 0;
double theta_1deg = 0;
double theta_2deg = 0;
double theta_3deg = 0;
double theta_4deg = 0;
int Channel1Position = 90;
int Channel2Position = 90;
int Channel3Position = 90;
int Channel3PrevPosition = 90;
double MaxReach = sqrt(pow(l_1,2)+pow(l_2,2)+pow(50*sin(phi),2));
double Current = 0;
void CurrentReach(void){
  Current = 115; // Work this out, like sqrt x^2+ y^2+z-77.6^2
}
void CalculateAngles(void){
  xp=sqrt(pow(x,2)+pow(y,2));
  zp=z-d_1;
  ctheta_1=x/xp;
  stheta_1=y/xp;
  theta_1 = atan2(stheta_1,ctheta_1);
  x3=x-l_3*sin(phi)*cos(theta_1);
  y3=y-l_3*sin(phi)*sin(theta_1);
  z3=z-l_3*cos(phi);
  x3p=sqrt(pow(x3,2)+pow(y3,2));
  z3p=z3-d_1;
  ctheta_3= ((pow(x3p,2))+(pow(z3p,2))-(pow(l_1,2))-(pow(l_2,2)))/(2*l_1*l_2);
  stheta_3= -sqrt(1-pow(ctheta_3,2));
  theta_3 = atan2(stheta_3,ctheta_3);
  sbeta= z3p/(sqrt((pow(x3p,2))+(pow(z3p,2))));
  cbeta = x3p/(sqrt((pow(x3p,2))+(pow(z3p,2))));
  beta = atan2(sbeta,cbeta);
  cpsi= ((pow(x3p,2))+(pow(z3p,2))+(pow(l_1,2))-(pow(l_2,2)))/(2*l_1*(sqrt((pow(x3p,2))+(pow(z3p,2)))));
  spsi= (l_2*stheta_3)/(sqrt((pow(x3p,2))+(pow(z3p,2))));
  psi= atan2(spsi,cpsi);
  theta_2 = beta-psi;
  theta_4= phi-theta_2-theta_3;
  //convert radians to degrees
  theta_1deg= theta_1*180/PI;
  theta_2deg= theta_2*180/PI;
  theta_3deg= theta_3*180/PI;
  theta_4deg= theta_4*180/PI;
    Serial.print(" Theta 1: ");
  Serial.print(theta_1deg);
    Serial.print(" Theta 2: ");
  Serial.print(theta_2deg);
    Serial.print(" Theta 3: ");
  Serial.print(theta_3deg);
    Serial.print(" Theta 4: ");
  Serial.print(theta_4deg);
  Serial.println();
}
void setup() {
  Serial.begin(115200);
  Servo1.attach(Channel1); 
  Servo2.attach(Channel2); 
  Servo3.attach(Channel3);
  Servo4.attach(Channel4);
  Servo5.attach(Channel5);
  Servo6.attach(Channel6);
//  attachPCINT(digitalPinToPCINT(4), MoveServo3, CHANGE);
//  attachPCINT(digitalPinToPCINT(5), MoveServo4, CHANGE);
//  attachPCINT(digitalPinToPCINT(6), MoveServo5, CHANGE);
//  attachPCINT(digitalPinToPCINT(7), MoveServo6, CHANGE);
}


void MoveServo1(void){
  Serial.print("Ch1: ");
  Serial.print(ch1.getAngle());
  Servo1Angle=theta_1deg;
  Servo1.write(Servo1Angle);
 
}
void MoveServo2(void){
  Serial.print(" Ch2: ");
  Serial.print(ch2.getAngle());
  Servo2Angle=theta_2deg;
  Servo2.write(Servo2Angle);
 
}
void MoveServo3(void){
    Serial.print(" Ch3: ");
  Serial.print(ch3.getAngle());
  Servo3Angle=theta_3deg;
  Servo3.write(Servo3Angle);
 
}
void MoveServo4(void){
      Serial.print(" Ch4: ");
  Serial.print(ch4.getAngle());
  Servo4Angle=theta_4deg;
  Servo4.write(Servo4Angle);
}
void MoveServo5(void){ //Rotate gripper
      Serial.print(" Ch5: ");
  Serial.print(ch5.getAngle());
  Servo5Angle=ch5.getAngle();
  Servo5.write(Servo5Angle);
}
void MoveServo6(void){ //activate gripper
      Serial.print(" Ch6: ");
  Serial.print(ch6.getAngle());
  Servo6Angle=ch6.getAngle();
  Servo6.write(Servo6Angle);
  }


void CheckXAxis(void){
  CurrentReach();
  Channel1Position = ch1.getAngle();
  
  if (Channel1Position>=105){
    if (x<MaxReach){ //Hopefully doesn't freak out if x is decimal just below reach
    x=x+1;
    }
  } else if (Channel1Position<=75){
    x=x-1;
  }
        Serial.print(" X is: ");
  Serial.print(x);
}
void CheckYAxis(void){
  CurrentReach();
  Channel2Position = ch2.getAngle();
  
  if (Channel2Position>=105){
    if (Current<MaxReach){ //Hopefully doesn't freak out if x is decimal just below reach
    y=y+1;
    }
  } else if (Channel2Position<=75){
    y=y-1;
  }
        Serial.print(" Y is ");
  Serial.print(y);
}
void CheckZAxis(void){
  CurrentReach();
  
  Channel3Position = ch3.getAngle();
  
  if (Channel3Position<=Channel3PrevPosition){
     z=Channel3Position;
  } else if (Channel3Position>Channel3PrevPosition && Current<MaxReach){
    z=Channel3Position;
  }
  Channel3PrevPosition=Channel3Position;
        Serial.print(" Z is ");
  Serial.print(z);
}
void loop() {
  CheckXAxis();
  CheckYAxis();
  CheckZAxis();
  CalculateAngles();
  MoveServo1();
  
  MoveServo2();
  MoveServo3();
  MoveServo4();
  MoveServo5();
  MoveServo6();
  Serial.println();
  delay(50);
}
