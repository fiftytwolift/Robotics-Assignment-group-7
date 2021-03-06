%% set up variables
close all
clear all
% timing, position and velocity for pt B and pt C
time = [3,8];
ptA = [2,5;0,0];
ptB = [5,8;0.25,0.25];
ptC = [14,7;0,0];
%% calculate the segments coefficents for x and y
x_coeff = TrajGen01([ptA(:,1),ptB(:,1)],[ptB(:,1),ptC(:,1)],time)
y_coeff = TrajGen01([ptA(:,2),ptB(:,2)],[ptB(:,2),ptC(:,2)],time)

%% get the displacement and velocity in both x and y
x_displacement = plot_displacement(x_coeff,time,true);
title('x displacement vs time')
y_displacement = plot_displacement(y_coeff,time,true);
title('y displacement vs time')
plot_velocity(x_coeff,time);
title('x velocity vs time')
plot_velocity(y_coeff,time);
title('y velocity vs time')


%% plot the final trajectory
figure()
hold on
plot(x_displacement,y_displacement)
pbaspect([14.5 8.5 1])
axis([0 14.5 0 8.5])
title('Trajectory for Task 1')
xlabel('x / m')
ylabel('y / m')
plot(ptA(1,1),ptA(1,2),'bo','LineWidth',5)
text(ptA(1,1)+0.5,ptA(1,2),'Pt A')
plot(ptB(1,1),ptB(1,2),'bo','LineWidth',5)
text(ptB(1,1)+0.5,ptB(1,2)-0.5,'Pt B')
plot(ptC(1,1),ptC(1,2),'bo','LineWidth',5)
text(ptC(1,1)+0.5,ptC(1,2),'Pt C')