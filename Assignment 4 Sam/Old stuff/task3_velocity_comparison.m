close all
clear all

figure('Name','velocity of robot')
hold on
title('direction of velocity of robot in task 3 vs task 2')
xlabel('t / s')
ylabel('direction / degree')

%% Initiate the variables for the trajectory
x_init = 0.71;
y_init = 1.08;
x_final = 1.485;
y_final = 0.041;

xdot_init = 0;
ydot_init = 0;
xdot_final = 0;
ydot_final = 0;
tfinal = 5.0;
dt = 0.01;
dt_PID = 0.001;
kp1 = 1;
kp2 = 1;
kd1 = 1500;
kd2 = 300;
%% robot parameters
m1 = 2.00;
m2 = 1;
Izz1 = 0.5;
Izz2 = 0.3;
L1 = 1;
L2 = 0.6;
rC1 = L1/2;
rC2 = L2/2;
g = 9.81;
%% Generate the coefficients for the two angles
tic;
x_eq_coeff = TrajGen_each_seg([x_init xdot_init],[x_final xdot_final] , [0 tfinal]);
y_eq_coeff = TrajGen_each_seg([y_init ydot_init],[y_final ydot_final] , [0 tfinal]);
toc;
fprintf('finished trajectory generation\n')
%% Generate the points on the trajectory
tic;
syms q1 q2
qdot_buffer =[];
time_control=0:dt:tfinal;
for time_index = 1: length(time_control)
    xdot_ref(time_index) = [3*time_control(time_index)^2 2*time_control(time_index) 1 0]*x_eq_coeff;
    ydot_ref(time_index) = [3*time_control(time_index)^2 2*time_control(time_index) 1 0]*y_eq_coeff;
    if ydot_ref(time_index) ~= 0
        hold on
        plot(time_control,atan2(ydot_ref(time_index),xdot_ref(time_index)));
    end
end
toc;
fprintf('finished calculating reference in joint space\n')



%% Initiate the variables for the trajectory
q1_init = 30;
q2_init = 75;
q1_final = -15;
q2_final = 45;

q1dot_init = 0;
q2dot_init = 0;
q1dot_final = 0;
q2dot_final = 0;
tfinal = 5.0;
dt = 0.01;
dt_PID = 0.001;
kp1 = 1;
kp2 = 1;
kd1 = 1500;
kd2 = 300;

%% robot parameters
m1 = 2.00;
m2 = 1;
Izz1 = 0.5;
Izz2 = 0.3;
L1 = 1;
L2 = 0.6;
rC1 = L1/2;
rC2 = L2/2;
g = 9.81;
%% Generate the coefficients for the two angles
q1_eq_coeff = TrajGen_each_seg([q1_init q1dot_init],[q1_final q1dot_final] , [0 tfinal]);
q2_eq_coeff = TrajGen_each_seg([q2_init q2dot_init],[q2_final q2dot_final] , [0 tfinal]);

%% Generate the points on the trajectory
time_control=0:dt:tfinal;
for time_index = 1: length(time_control)
    q1_ref(time_index) = [time_control(time_index)^3 time_control(time_index)^2 time_control(time_index) 1]*q1_eq_coeff;
    q2_ref(time_index) = [time_control(time_index)^3 time_control(time_index)^2 time_control(time_index) 1]*q2_eq_coeff;
    q1dot_ref(time_index) = [3*time_control(time_index)^2 2*time_control(time_index) 1 0]*q1_eq_coeff;
    q2dot_ref(time_index) = [3*time_control(time_index)^2 2*time_control(time_index) 1 0]*q2_eq_coeff;
    %Calculate the actual x,y velocity
    if q2_ref(time_index)~= 0
        xy_vel_real = round(double(subs(J_v,[q1_ref(time_index) q2_ref(time_index)],[q1_ref(time_index)*180/pi q2_ref(time_index)*180/pi])),3)*[q1dot_ref(time_index);q2dot_ref(time_index)];
        hold on
        plot(time_control, atan2(xy_vel_real(2),xy_vel_real(1)));
    end
end
