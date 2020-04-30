%% draw the obstacle circle
r = 1.5;
circle_xy = [10 7];
index = 0;
for i = 0:0.01:3*pi
    index = index+1;
    circle_x(index) = 10 + r*cos(i);
    circle_y(index) = 7 +  r*sin(i);
end
%% define point C
ptC = [14,7;0,0];

%% plot the final trajectory with robotic arm
figure()
hold on
plot(circle_x,circle_y,'-')
[Q1,Q2] = two_arm_IK(14,7,true);
plot_robot(Q1,Q2);
[Q1,Q2] = two_arm_IK(14,7,false);
plot_robot(Q1,Q2);
pbaspect([14.5 10.5 1])
axis([0 14.5 -2 8.5])
xlabel('x / m')
ylabel('y / m')
title('Elbow up and elbow down reaching point C')
plot(ptC(1,1),ptC(1,2),'bo','LineWidth',5)
text(ptC(1,1)+0.5,ptC(1,2),'Pt C')
