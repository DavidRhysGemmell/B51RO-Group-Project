//Waypoint mission for robotic arm


#include <Servo.h>  
#define Channel1 8
#define Channel2 9
#define Channel3 10
#define Channel4 11
#define Channel5 12
#define Channel6 13



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


double d_1 = 150; //Height to first joint
int l_1=150; // Lenght of first arm
int l_2=150;  // Length of second arm
int l_3=150; // Length of third arm
int RobotBody= 0; //
int x = 150; //Initalise End coordinates
int y = 100;
int z = 87;

float phi = -PI/2; // End posture of arm, in radians. Could be editable.

float xp = 0;
float zp = 0;
float ctheta_1 = 0;
float stheta_1 = 0;
float theta_1 = 90;
float x3 = 0;
float y3 = 0;
float z3 = 0;

double ctheta_3 = 0;
double stheta_3 = 0;
double theta_3 = 45;
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
int Channel2Position = 45;
int Channel3Position = 45;
int Channel3PrevPosition = 45;
double MaxReach = 295;
double Current = 0;

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

void CurrentReach(void){
  Current = sqrt(pow(xp,2)+pow(zp,2)); // Work this out, like sqrt x^2+ y^2+z-77.6^2
}
void CalculateAngles(void){
  //Theta 1 calculation
  //Calculate the joint before the end one
xp=x-l_3*cos(phi);
zp=z-l_3*sin(phi);


  
  ctheta_3= ((pow(xp,2))+(pow(zp,2))-(pow(l_1,2))-(pow(l_2,2)))/(45000);
  stheta_3= -sqrt(1-pow(ctheta_3,2));
  theta_3 = atan2(stheta_3,ctheta_3);
  sbeta= zp/(sqrt((pow(xp,2))+(pow(zp,2))));
  cbeta = xp/(sqrt((pow(xp,2))+(pow(zp,2))));
  beta = atan2(sbeta,cbeta);
  cpsi= ((pow(xp,2))+(pow(zp,2))+(pow(l_1,2))-(pow(l_2,2)))/(2*l_1*(sqrt((pow(xp,2))+(pow(zp,2)))));
  spsi= (l_2*stheta_3)/(sqrt((pow(xp,2))+(pow(zp,2))));
  psi= atan2(spsi,cpsi);
  theta_2 = beta-psi;
  theta_4= phi-theta_2-theta_3;

  //convert radians to degrees
  theta_2deg= 180-(theta_2*180/PI);
  theta_3deg= 180+(theta_3*180/PI);
  theta_4deg= -(theta_4*180/PI);

    Serial.print(" Theta 2: ");
  Serial.print(theta_2deg);
    Serial.print(" Theta 3: ");
  Serial.print(theta_3deg);
    Serial.print(" Theta 4: ");
  Serial.print(theta_4deg);
  Serial.println();
}
void MoveServo1(void){

  Servo1Angle=y;
  Servo1.write(Servo1Angle);
 
}
void MoveServo2(void){

  Servo2Angle=theta_2deg;
  Servo2.write(theta_2deg);
 
}
void MoveServo3(void){

  Servo3Angle=theta_3deg;
  Servo3.write(theta_3deg);
 
}
void MoveServo4(void){

  Servo4Angle=theta_4deg;
  Servo4.write(theta_4deg);
}



void StartPosition(void) {
x=149;
y=74;
z=45;
}

void AbovePin(void) {
x=139;
y=116;
z=6;
}
void OnPin(void) {
x=139;
y=116;
z=-30;
}
void AboveAbovePin(void) {
x=139;
y=116;
z=78;
}
void AboveHolder(void) {
x=139;
y=34;
z=43;
}

void InHolder(void) {
x=139;
y=34;
z=-4
;
}
void Face(void) {
x=24;
y=25;
z=48;
}
void Grin(void) {
x=24;
y=25;
z=48;
//phi=PI;
}

void Move(void){
    CalculateAngles();
  MoveServo1(); 
  MoveServo2();
  MoveServo3();
  MoveServo4();

}

void OpenGripper(void){
  Servo6.write(0);
}
void CloseGripper(void){
  Servo6.write(180);
}

void loop() {
phi = -PI/2;
Servo5.write(120);
Servo6.write(25);
 delay(2000);
 StartPosition();
 Move();
 delay(5000);
 Servo5.write(135);
 AbovePin();
 Move();
 delay(2000);
 OnPin();
 Move();
 delay(500);
  CloseGripper();
 delay(500);
 AboveAbovePin();
 Move();
 delay(1500);
 AboveHolder();
 Move();
 delay(2000);
 InHolder();
 Move();
 delay(500);
 OpenGripper();
 delay(500);
 Face();
 Move();
 delay(500);
 Grin();
 Move();
 Servo4.write(0);
 delay(500);
 Servo5.write(150);
 Servo6.write(40);
delay(5000);



}
