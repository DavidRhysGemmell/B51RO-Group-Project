clear all
close all
%-------------------------------------
%Inverse kinematics fo Planar 3R robot
% used in Module B59RM (B59DF)
%Author: Xianwen Kong
%-------------------------------------
% List of functions used: atan2, sqrt, sin, cos, plot
% for help on the above funstions please use help followed bz the function.
% stheta_2 represents sin(theta_2), …
% ctheta_2 represents cos(theta_2), …
%input link parameter
l_1=150;
l_2=150;
l_3=150;
%input the pose of the endeffector
x=147;
z=-27;
phi=pi;
%------------------------------------
%plot the last link of the robot
%---------------------------------------------------------
% calcutatin the coordinates of joint centers and point P
O_3x=x-l_3*cos(phi);
O_3z=z-l_3*sin(phi);
% plot the robot
XX=[O_3x x];
ZZ=[O_3z z];
plot(XX,ZZ,'r','LineWidth',6);
hold on; % This allows addition lines to be drawn in the above figure
xp=x-l_3*cos(phi);
zp=z-l_3*sin(phi);
% calculate theta_2
%-----------------complete the following ---------------------------------
ctheta_2= ((xp^2)+(zp^2)-(l_1^2)-(l_2^2))/(2*l_1*l_2); % complete this line
% elbow down configuration
stheta_2 = -(1-(ctheta_2)^2)^(1/2); % elbow down configuration, % complete this line
% for elbow up configuration use
%stheta_2 = ; % elbow up configuration, % complete this line
% calculate beta
theta_2 = atan2(stheta_2,ctheta_2);
sbeta= zp/(((xp^2)+(zp^2))^(1/2)); % complete this line
cbeta = xp/(((xp^2)+(zp^2))^(1/2)); % complete this line
beta = atan2(sbeta,cbeta); % complete this line
% calculate psi
cpsi= ((xp^2)+(zp^2)+(l_1^2)-(l_2^2))/(2*l_1*(((xp^2)+(zp^2))^(1/2))); % complete this line
spsi= (l_2*stheta_2)/(((xp^2)+(zp^2))^(1/2)); % complete this line
psi= atan2(spsi,cpsi); % complete this line
theta_1= beta-psi; % complete this line
% calculate theta_3
theta_3= phi-theta_1-theta_2;% complete this line
%---------------------------------------------------------
%plot the robot
%---------------------------------------------------------
% calculating the coordinates of joint centers and point P 
% O_1x and O_1z represent the coordinates of joint center O1
O_1x=0;
O_1z=0;
O_2x=O_1x+l_1*cos(theta_1);
O_2z=O_1z+l_1*sin(theta_1);
O_3x=O_2x+l_2*cos(theta_1+theta_2); % complete this line
O_3z=O_2z+l_2*sin(theta_1+theta_2); % complete this line
Px=O_3x+l_3*cos(theta_1+theta_2+theta_3);
Pz=O_3z+l_3*sin(theta_1+theta_2+theta_3);
% plot the robot
XX=[O_1x O_2x O_3x Px];
ZZ=[O_1z O_2z O_3z Pz];
XXj=[O_1x O_2x O_3x];
ZZj=[O_1z O_2z O_3z];
plot(XXj,ZZj,'o','LineWidth',2,'MarkerSize',10);
plot(XX,ZZ,'g','LineWidth',2);
%hold off
theta_1deg=180-(theta_1*180/pi)
theta_2deg=180+(theta_2*180/pi)
theta_3deg=-(theta_3*180/pi)
