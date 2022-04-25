close all
clear all
d_1=150;
l_1=150;
l_2=150;
l_3=150;
phi=-pi/2;
theta_1 = 0; %input theta_1
theta_2 = 75*pi/180;
theta_3 = 22*pi/180;
theta_4 = 90*pi/180;
%---------------------------------------------------------
%plot the robot based on forward kinematics
%---------------------------------------------------------
% calcutatin the coordinates of joint centers and point P
O_1x=0;
O_1y=0;
O_1z=0;
O_2x=0;
O_2y=0;
O_2z=0;
O_3x=O_2x+l_1*cos(theta_2)*cos(theta_1);
O_3y=O_2y+l_1*cos(theta_2)*sin(theta_1);
O_3z=O_2z+l_1*sin(theta_2);
O_4x= O_3x+l_2*cos(theta_3+theta_2)*cos(theta_1);%complete this line
O_4y= O_3y+l_2*cos(theta_3+theta_2)*sin(theta_1);%complete this line
O_4z= O_3z+l_2*sin(theta_2+theta_3);%complete this line
Px= O_4x+l_3*sin(theta_4+theta_3+theta_2)*cos(theta_1);%complete this line
Py= O_4y+l_3*sin(theta_4+theta_3+theta_2)*sin(theta_1);%complete this line
Pz= O_4z+l_3*cos(theta_2+theta_3+theta_4);%complete this line
% plot the robot
XX=[O_1x O_2x O_3x O_4x Px];
YY=[O_1y O_2y O_3y O_4y Py];
ZZ=[O_1z O_2z O_3z O_4z Pz];
XXj=[O_1x O_2x O_3x O_4x Px];
YYj=[O_1y O_2y O_3y O_4y Py];
ZZj=[O_1z O_2z O_3z O_4z Pz];
plot3(XXj,YYj,ZZj,'o','LineWidth',2,'MarkerSize',10);
hold on;
plot3(XX,YY,ZZ, 'g','LineWidth',2);
hold off;
