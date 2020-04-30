function plot_robot(Q1,Q2,color_shift)
% length of the arm
L1 = 9;
L2 = 9;
% points of joints at the robot
mid_pt = [L1*cosd(Q1),L1*sind(Q1)];
end_effector = mid_pt+[L2*cosd(Q1+Q2),L2*sind(Q1+Q2)];

x = [0 mid_pt(1) end_effector(1)];
y = [0 mid_pt(2) end_effector(2)];
% %% for plotting the points of the joints
% plot(mid_pt(1),mid_pt(2),'o','LineWidth',5,'Color','black')
% text(mid_pt(1),mid_pt(2)-0.5,'Pt G','FontWeight','bold')
% plot(end_effector(1),end_effector(2),'o','LineWidth',5,'Color','black')
% text(end_effector(1),end_effector(2)-0.5,'Pt T')
% plot(0,0,'o','LineWidth',5,'Color','black')
% text(0,0-0.5,'Pt F')
hold on
if nargin==3
    plot(x,y,'Color',[0.85-color_shift/2 0.325 0.0980+color_shift],'LineWidth',3);
elseif nargin==2
    plot(x,y,'LineWidth',3);
end