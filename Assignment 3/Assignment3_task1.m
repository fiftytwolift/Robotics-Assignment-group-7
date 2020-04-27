%% set up variables
close all
clear all
% timing, position and velocity for pt B and pt C
time = [0,3,8];
ptA = [2,5;0,0];
ptB = [5,8;0.25,0.25];
ptC = [14,7;0,0];
%% calculate the segments coefficents for x and y
x_coeff_1 = TrajGen01(ptA(:,1),ptB(:,1),time(1:2));
x_coeff_2 = TrajGen01(ptB(:,1),ptC(:,1),time(2:3));
y_coeff_1 = TrajGen01(ptA(:,2),ptB(:,2),time(1:2));
y_coeff_2 = TrajGen01(ptB(:,2),ptC(:,2),time(2:3));
x_coeff = [x_coeff_1;x_coeff_2];
y_coeff = [y_coeff_1;y_coeff_2];
%% get the displacement and velocity in both x and y
x_displacement = plot_displacement(x_coeff,time(2:3),true);
y_displacement = plot_displacement(y_coeff,time(2:3),true);
plot_velocity(x_coeff,time(2:3));
plot_velocity(y_coeff,time(2:3));


%% plot the final trajectory
figure()
hold on
plot(x_displacement,y_displacement)
pbaspect([14 8.5 1])
axis([0 14 0 8.5])
xlabel('x / m')
ylabel('y / m')