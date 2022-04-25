close all
clear all
%-------------------------------------
%Inverse kinematics fo Spatial 3R robot
%Author: David Rhys Gemmell
% Date 25/04/2022
%-------------------------------------
% List of functions used: atan2, sqrt, sin, cos, plot
% for help on the above funstions please use help followed by the function.
%input link parameter
d_1=150;
l_1=150;
l_2=150;
l_3=150;

%input the pose of the endeffector
x=180;
y=0;
z=0;
phi=pi;

x4=x;
y4=y;
z4=z;



%------------------------------------
%plot the last link of the robot
%---------------------------------------------------------
% calcutatin the coordinates of joint centers and point P
%O_4x=x-l_3*cos(phi);
%O_4z=z-l_3*sin(phi);
% calculate theta_1
xp=sqrt(x^2+y^2);
zp=z-d_1;
ctheta_1=x/xp;
stheta_1=y/xp; %complete this line
theta_1 = atan2(stheta_1,ctheta_1);

x3=x4-l_3*sin(phi)*cos(theta_1);
y3=y4-l_3*sin(phi)*sin(theta_1);
z3=z4-l_3*cos(phi);

x3p=sqrt(x3^2+y3^2);
z3p=z3-d_1;
%xp=x+l_3*cos(phi);
%zp=z+l_3*cos(phi);
% calculate theta_3
%-----------------complete the following ---------------------------------
ctheta_3= ((x3p^2)+(z3p^2)-(l_1^2)-(l_2^2))/(2*l_1*l_2); % complete this line
% elbow down configuration
stheta_3= -(1-(ctheta_3)^2)^(1/2); % elbow down configuration, % complete this line
% for elbow up configuration use
%stheta_2 = ; % elbow up configuration, % complete this line

% calculate beta
theta_3 = atan2(stheta_3,ctheta_3);
sbeta= z3p/(((x3p^2)+(z3p^2))^(1/2)); % complete this line
cbeta = x3p/(((x3p^2)+(z3p^2))^(1/2)); % complete this line
beta = atan2(sbeta,cbeta); % complete this line


% calculate psi
cpsi= ((x3p^2)+(z3p^2)+(l_1^2)-(l_2^2))/(2*l_1*(((x3p^2)+(z3p^2))^(1/2))); % complete this line
spsi= (l_2*stheta_3)/(((x3p^2)+(z3p^2))^(1/2)); % complete this line
psi= atan2(spsi,cpsi); % complete this line
theta_2 = beta-psi; % complete this line

% calculate theta_4
theta_4= phi-theta_2-theta_3;% complete this line
%---------------------------------------------------------
%plot the robot based on forward kinematics
%---------------------------------------------------------
% calcutatin the coordinates of joint centers and point P
O_1x=0;
O_1y=0;
O_1z=0;
O_2x=0;
O_2y=0;
O_2z=d_1;
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
XXj=[O_1x O_2x O_3x O_4x Px x];
YYj=[O_1y O_2y O_3y O_4y Py y];
ZZj=[O_1z O_2z O_3z O_4z Pz z];
plot3(XXj,YYj,ZZj,'o','LineWidth',2,'MarkerSize',10);
hold on;
plot3(XX,YY,ZZ, 'g','LineWidth',2);
hold off;
