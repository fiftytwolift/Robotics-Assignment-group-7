close all
clear all
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
kq1 = 5;
kp = 300;
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
    x_ref(time_index) = [time_control(time_index)^3 time_control(time_index)^2 time_control(time_index) 1]*x_eq_coeff;
    y_ref(time_index) = [time_control(time_index)^3 time_control(time_index)^2 time_control(time_index) 1]*y_eq_coeff;
    xdot_ref(time_index) = [3*time_control(time_index)^2 2*time_control(time_index) 1 0]*x_eq_coeff;
    ydot_ref(time_index) = [3*time_control(time_index)^2 2*time_control(time_index) 1 0]*y_eq_coeff;
    % convert the taskspace reference into jointspace reference
    [q1_buffer, q2_buffer] =  two_arm_IK(x_ref(time_index),y_ref(time_index),false);
    q1_ref(time_index) = q1_buffer;
    q2_ref(time_index) = q2_buffer;
    if time_index==1 || time_index == length(time_control)
        qdot_buffer = [0,0];
    else
        qdot_buffer = [((q1_ref(time_index)-q1_ref(time_index-1))/dt),((q2_ref(time_index)-q2_ref(time_index-1))/dt)];
    end
    q1dot_ref(time_index) = round(double(qdot_buffer(1)),4);
    q2dot_ref(time_index) = round(double(qdot_buffer(2)),4);
end
toc;
fprintf('finished calculating reference in joint space\n')
%% Define the initial position in joint space
q1_init = q1_ref(1);
q2_init = q2_ref(1);
%% PID controller
% Initialise the robot to the initial position and velocity
% q for control loop
q(1)= q1_init/180*pi; % Converting degree to radian
q(2) = q2_init/180*pi;
qdot(1) = 0;
qdot(2) = 0;
% Limited by the physical limit
[q(1),q(2),qdot(1),qdot(2)] = Task3_angle_limit(q(1),q(2),qdot(1),qdot(2));

%q for output
q1s(1) = q(1)/pi*180; % Output uses degree directly
q2s(1) = q(1)/pi*180;
qdot1s(1) = 0;
qdot2s(1) = 0;

i = 1;
PID_i = 1;
tau=[0;0];
tic
for time=0:dt_PID:tfinal
    % Error term in angular velocity
    % The reference signal is lagging behind the actual trajectory
    q1_e_diff(i) = q1dot_ref(floor(i/(dt/dt_PID))+1)/180*pi - qdot(1);
    q2_e_diff(i) = q2dot_ref(floor(i/(dt/dt_PID))+1)/180*pi - qdot(2);
    
    q1dot_c = kp*q1_e_diff(i);
    q2dot_c = kp*q2_e_diff(i);
    %Torque is based on counter-gravity torque + control signal
    tau = [ (g*m2*(rC2*cos(q(1) + q(2)) + L1*cos(q(1))) + g*m1*rC1*cos(q(1))) + kq1*q1dot_c;
        (g*m2*rC2*cos(q(1) + q(2))) + q2dot_c];
    
    [t,y] = ode45(@(t,y) runrobot(t,y,tau), [0, dt_PID], [q(1), q(2), qdot(1), qdot(2)]);
    Ly = length(y(:,1));
    q(1) = y(Ly,1);
    q(2) = y(Ly,2);
    qdot(1) = y(Ly,3);
    qdot(2) = y(Ly,4);
    % Limited by the physical limit
    [q(1),q(2),qdot(1),qdot(2)] = Task3_angle_limit(q(1),q(2),qdot(1),qdot(2));
    
    q1s(i) = q(1)*180/pi;
    q2s(i) = q(2)*180/pi;
    qdot1s(i) = qdot(1)*180/pi;
    qdot2s(i) = qdot(2)*180/pi;
    torque(i,:) = tau;
    
    i = i+1;
end
toc;
fprintf('finished Traj A simulation\n')
%% Plotting the angles and angular velocities
figure()
subplot(2,2,1)
hold on
plot([0:dt_PID:tfinal],q1s,'r')
plot([0:dt:tfinal],q1_ref,'b')
xlabel('Time/s')
ylabel('Angle/Degree')
title('q1 vs time ')

subplot(2,2,2)
hold on
plot([0:dt_PID:tfinal],qdot1s,'r')
plot([0:dt:tfinal],q1dot_ref,'b')
xlabel('Time/s')
ylabel('Angular velocity /Deg per sec')
title('q1dot vs time ')

subplot(2,2,3)
hold on
plot([0:dt_PID:tfinal],q2s,'r')
plot([0:dt:tfinal],q2_ref,'b')
xlabel('Time/s')
ylabel('Angle/Degree')
title('q2 vs time')

subplot(2,2,4)
hold on
plot([0:dt_PID:tfinal],qdot2s,'r')
plot([0:dt:tfinal],q2dot_ref,'b')
xlabel('Time/s')
ylabel('Angular velocity /Deg per sec')
title('q2dot vs time')

%% Plotting the torques
figure()
hold on
plot([0:dt_PID:tfinal],torque(:,1),'r')
plot([0:dt_PID:tfinal],torque(:,2),'b')
xlabel('Time/s')
ylabel('Torque/Nm')
legend('Torque for joint 1','Torque for joint 2')
title('Torque for joint 1 and joint 2')

%% Plotting the robot
figure()
for t = 0:tfinal/10:tfinal
    extractedQ1 = q1s(floor(t/dt_PID)+1);
    extractedQ2 = q2s(floor(t/dt_PID)+1);
    plot_robot(extractedQ1,extractedQ2);
end
title('Trajectory A in taskspace splined, jointspace velocity control')
xlabel('x / m')
ylabel('y / m')

%% Trajectory B
%% Initiate the variables for the trajectory
x_init = 0.71;
y_init = 1.08;
x_final = 1.36;
y_final = -0.59;

xdot_init = 0;
ydot_init = 0;
xdot_final = 0;
ydot_final = 0;
tfinal = 5.0;
dt = 0.01;
dt_PID = 0.001;
kq1 = 5;
kp = 300;
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
    x_ref(time_index) = [time_control(time_index)^3 time_control(time_index)^2 time_control(time_index) 1]*x_eq_coeff;
    y_ref(time_index) = [time_control(time_index)^3 time_control(time_index)^2 time_control(time_index) 1]*y_eq_coeff;
    xdot_ref(time_index) = [3*time_control(time_index)^2 2*time_control(time_index) 1 0]*x_eq_coeff;
    ydot_ref(time_index) = [3*time_control(time_index)^2 2*time_control(time_index) 1 0]*y_eq_coeff;
    % convert the taskspace reference into jointspace reference
    [q1_buffer, q2_buffer] =  two_arm_IK(x_ref(time_index),y_ref(time_index),false);
    q1_ref(time_index) = q1_buffer;
    q2_ref(time_index) = q2_buffer;
    if time_index==1 || time_index == length(time_control)
        qdot_buffer = [0,0];
    else
        qdot_buffer = [((q1_ref(time_index)-q1_ref(time_index-1))/dt),((q2_ref(time_index)-q2_ref(time_index-1))/dt)];
    end
    q1dot_ref(time_index) = round(double(qdot_buffer(1)),4);
    q2dot_ref(time_index) = round(double(qdot_buffer(2)),4);
end
toc;
fprintf('finished calculating reference in joint space\n')
%% Define the initial position in joint space

%% PID controller
% Initialise the robot to the initial position and velocity
% q for control loop
q(1)= q1_init/180*pi; % Converting degree to radian
q(2) = q2_init/180*pi;
qdot(1) = 0;
qdot(2) = 0;
% Limited by the physical limit
[q(1),q(2),qdot(1),qdot(2)] = Task3_angle_limit(q(1),q(2),qdot(1),qdot(2));

%q for output
q1s(1) = q(1)/pi*180; % Output uses degree directly
q2s(1) = q(1)/pi*180;
qdot1s(1) = 0;
qdot2s(1) = 0;

i = 1;
PID_i = 1;
tau=[0;0];
tic
for time=0:dt_PID:tfinal
    % Error term in angular velocity
    % The reference signal is lagging behind the actual trajectory
    q1_e_diff(i) = q1dot_ref(floor(i/(dt/dt_PID))+1)/180*pi - qdot(1);
    q2_e_diff(i) = q2dot_ref(floor(i/(dt/dt_PID))+1)/180*pi - qdot(2);
    
    q1dot_c = kp*q1_e_diff(i);
    q2dot_c = kp*q2_e_diff(i);
    %Torque is based on counter-gravity torque + control signal
    tau = [ (g*m2*(rC2*cos(q(1) + q(2)) + L1*cos(q(1))) + g*m1*rC1*cos(q(1))) + kq1*q1dot_c;
        (g*m2*rC2*cos(q(1) + q(2))) + q2dot_c];
    
    [t,y] = ode45(@(t,y) runrobot(t,y,tau), [0, dt_PID], [q(1), q(2), qdot(1), qdot(2)]);
    Ly = length(y(:,1));
    q(1) = y(Ly,1);
    q(2) = y(Ly,2);
    qdot(1) = y(Ly,3);
    qdot(2) = y(Ly,4);
    
    % Limited by the physical limit
    [q(1),q(2),qdot(1),qdot(2)] = Task3_angle_limit(q(1),q(2),qdot(1),qdot(2));    
    q1s(i) = q(1)*180/pi;
    q2s(i) = q(2)*180/pi;
    qdot1s(i) = qdot(1)*180/pi;
    qdot2s(i) = qdot(2)*180/pi;
    torque(i,:) = tau;    
    i = i+1;
end
toc;
fprintf('finished Traj B simulation\n')

%% Plotting the robot
figure()
for t = 0:tfinal/10:tfinal
    extractedQ1 = q1s(floor(t/dt_PID)+1);
    extractedQ2 = q2s(floor(t/dt_PID)+1);
    plot_robot(extractedQ1,extractedQ2);
end
title('Trajectory B in taskspace splined, jointspace velocity control')
xlabel('x / m')
ylabel('y / m')
axis([0 1.4 -0.8 1.2]);
pbaspect([14 20 1]);

