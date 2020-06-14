close all
clear all
L1 = 10;
L3 = 25;
L4 = 15;
parallel_angle = atand(L3/L1);


% pointing to point w when q1 = 0
q1 = 0;
xw = L4 + sqrt(L1^2+L3^2); 
yw = 0;
% pointing to right when q1 = 0
% xw = L4 + L3; 
% yw = L1;
% q1 = 0;
% q3 = 0;
% pointing to top when q1 = 0
% q1 = -30;
% xw = L4 + L1; 
% yw = -L3;
% xb = xw*cosd(q3)-yw*sind(q3)
% yb = xw*sind(q3)+yw*cosd(q3)
%% inverse for q3
q3 = acosd((-L1*yw+L3*L4-L3*xw)/(2*L4*xw-L4^2-xw^2-yw^2))

%% calculate and plot the points
point1 = [0,0];
point2 = [-L1*sind(q1),L1*cosd(q1)];
point3 = point2 + [L3*cosd(q1),L3*sind(q1)];
point4 = point3 + [L4*cosd(q1+q3),L4*sind(q1+q3)];

x = [point1(1),point2(1),point3(1),point4(1)];
y = [point1(2),point2(2),point3(2),point4(2)];

plot(x,y,'linewidth',5)
axis([-40 40 0 40])
pbaspect([2 1 1]);
