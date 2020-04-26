%% set up variables
close all
clear all
tfinal = [3,5.5,8];
d_xpos = 10;
d_xvel = 0.25;
d_ypos = 5;
d_yvel = 0.25;

%% calculate the segments coefficents for x and y
x_coeff = TrajGen([2,0],[14,0],[5,0.25,d_xpos,d_xvel],tfinal);
y_coeff = TrajGen([5,0],[7,0],[8,0.25,d_ypos,d_yvel],tfinal);

%% get the displacement and velocity in both x and y
hold off
x_displacement = plot_displacement(x_coeff,tfinal);
y_displacement = plot_displacement(y_coeff,tfinal);
plot_velocity(x_coeff,tfinal);
plot_velocity(y_coeff,tfinal);

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
plot(circle_x,circle_y,'o')
plot(x_displacement,y_displacement)
skip_frame = 10;
for i = 0:skip_frame:length(Q1)-1
    plot_robot(Q1(i+1),Q2(i+1));
end 
pbaspect([14 13.5 1])
axis([0 14 -5 8.5])
xlabel('x / m')
ylabel('y / m')