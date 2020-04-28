function plot_robot(Q1,Q2,color_shift)
% length of the arm
L1 = 9;
L2 = 9;
% points of joints at the robot
mid_pt = [L1*cosd(Q1),L1*sind(Q1)];
end_effector = mid_pt+[L2*cosd(Q1+Q2),L2*sind(Q1+Q2)];

x = [0 mid_pt(1) end_effector(1)];
y = [0 mid_pt(2) end_effector(2)];
hold on
plot(x,y,'Color',[0.85-color_shift/2 0.325 0.0980+color_shift],'LineWidth',3);