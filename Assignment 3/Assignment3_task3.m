%% set up variables
close all
clear all

% timing, position and velocity for pt B, pt C and pt D
time = [0,3,5.5,8];
ptA = [2,5;0,0];
ptB = [5,8;0.25,0.25];
ptC = [14,7;0,0];
ptD = [9.9,5.3;0,0];
%% calculate the segments coefficents for x and y
x_coeff_1 = TrajGen01(ptA(:,1),ptB(:,1),time(1:2));
x_coeff_2 = TrajGen01(ptB(:,1),ptD(:,1),time(2:3));
x_coeff_3 = TrajGen01(ptD(:,1),ptC(:,1),time(3:4));
y_coeff_1 = TrajGen01(ptA(:,2),ptB(:,2),time(1:2));
y_coeff_2 = TrajGen01(ptB(:,2),ptD(:,2),time(2:3));
y_coeff_3 = TrajGen01(ptD(:,2),ptC(:,2),time(3:4));
x_coeff = [x_coeff_1;x_coeff_2;x_coeff_3];
y_coeff = [y_coeff_1;y_coeff_2;y_coeff_3];

%% get the displacement and velocity in both x and y
x_displacement = plot_displacement(x_coeff,time(2:4),true);
y_displacement = plot_displacement(y_coeff,time(2:4),true);
plot_velocity(x_coeff,time(2:4));
plot_velocity(y_coeff,time(2:4));

%% get the robotic arm
Q1 = [];
Q2 = [];
for i = 1:length(x_displacement)
    [Q1(i),Q2(i)] = two_arm_IK(x_displacement(i),y_displacement(i));
end

%% draw the obstacle circle
r = 1.5;
circle_xy = [10 7];
index = 0;
for i = 0:0.01:3*pi
    index = index+1;
    circle_x(index) = 10 + r*cos(i);
    circle_y(index) = 7 +  r*sin(i);
end

%% check whether we collided with the obstacle
closest_dist_to_obs = inf; 

for i = 1:length(x_displacement)
    dist = sqrt( (x_displacement(i) - circle_xy(1))^2 + (y_displacement(i) - circle_xy(2))^2);
    if dist < closest_dist_to_obs 
        closest_dist_to_obs = dist;
    end
end
if closest_dist_to_obs < r
    warning("the trajectory collided with the obstacle")
    return
end

%% plot the final trajectory with robotic arm
figure()
hold on
plot(circle_x,circle_y,'-')
plot(x_displacement,y_displacement)
skip_frame = 10;
for i = 0:skip_frame:length(Q1)-1
    plot_robot(Q1(i+1),Q2(i+1));
end 
pbaspect([14 13.5 1])
axis([0 14 -5 8.5])
xlabel('x / m')
ylabel('y / m')