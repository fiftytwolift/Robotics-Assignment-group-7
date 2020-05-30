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
kp1 = 1250;
kp2 = 380;
kd1 = 0.05;
kd2 = 0.02;
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
q2s(1) = q(2)/pi*180;
qdot1s(1) = 0;
qdot2s(1) = 0;

i = 2;
tau=[0;0];
tic
for time=dt_PID:dt_PID:tfinal
    % Error term in angular position
    q1_diff(i) = q1dot_ref(ceil(i/(dt/dt_PID)))/180*pi - qdot(1);
    q2_diff(i) = q2dot_ref(ceil(i/(dt/dt_PID)))/180*pi - qdot(2);
    
    % Error term in angular velocity
    q1_e_diff(i) = (q1dot_ref(ceil(i/(dt/dt_PID)))-q1dot_ref(max(ceil((i-1)/(dt/dt_PID)),1)))/dt-(q1_diff(i)-q1_diff(max(i-1,1)))/dt_PID;
    q2_e_diff(i) = (q2dot_ref(ceil(i/(dt/dt_PID)))-q2dot_ref(max(ceil((i-1)/(dt/dt_PID)),1)))/dt-(q2_diff(i)-q2_diff(max(i-1,1)))/dt_PID;
    
    % control signal for each joint
    q1dot_c = kp1*q1_diff(i) + kd1*q1_e_diff(i);
    q2dot_c = kp2*q2_diff(i) + kd2*q2_e_diff(i);   
    %Torque is based on counter-gravity torque + control signal
    tau = [ (g*m2*(rC2*cos(q(1) + q(2)) + L1*cos(q(1))) + g*m1*rC1*cos(q(1))) + q1dot_c;
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
kp1 = 1250;
kp2 = 380;
kd1 = 0.05;
kd2 = 0.02;
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
    x_ref_b(time_index) = [time_control(time_index)^3 time_control(time_index)^2 time_control(time_index) 1]*x_eq_coeff;
    y_ref_b(time_index) = [time_control(time_index)^3 time_control(time_index)^2 time_control(time_index) 1]*y_eq_coeff;
    xdot_ref(time_index) = [3*time_control(time_index)^2 2*time_control(time_index) 1 0]*x_eq_coeff;
    ydot_ref(time_index) = [3*time_control(time_index)^2 2*time_control(time_index) 1 0]*y_eq_coeff;
    % convert the taskspace reference into jointspace reference
    [q1_buffer, q2_buffer] =  two_arm_IK(x_ref_b(time_index),y_ref_b(time_index),false);
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
%% PD controller 
% Initialise the robot to the initial position and velocity
% q for control loop
q(1)= q1_init/180*pi; % Converting degree to radian
q(2) = q2_init/180*pi;
qdot(1) = 0;
qdot(2) = 0;
% Limited by the physical limit
[q(1),q(2),qdot(1),qdot(2)] = Task3_angle_limit(q(1),q(2),qdot(1),qdot(2));

%q for output
q1s_b(1) = q(1)/pi*180; % Output uses degree directly
q2s_b(1) = q(2)/pi*180;
qdot1s_b(1) = 0;
qdot2s_b(1) = 0;

i = 2;
tau=[0;0];
tic
for time=dt_PID:dt_PID:tfinal
    % Error term in angular velocity
    q1_diff(i) = q1dot_ref(ceil(i/(dt/dt_PID)))/180*pi - qdot(1);
    q2_diff(i) = q2dot_ref(ceil(i/(dt/dt_PID)))/180*pi - qdot(2);
    
    % Error term in angular acceleration
    q1_e_diff(i) = (q1dot_ref(ceil(i/(dt/dt_PID)))-q1dot_ref(max(ceil((i-1)/(dt/dt_PID)),1)))/dt-(q1_diff(i)-q1_diff(max(i-1,1)))/dt_PID;
    q2_e_diff(i) = (q2dot_ref(ceil(i/(dt/dt_PID)))-q2dot_ref(max(ceil((i-1)/(dt/dt_PID)),1)))/dt-(q2_diff(i)-q2_diff(max(i-1,1)))/dt_PID;
    
    % control signal for each joint
    q1dot_c = kp1*q1_diff(i) + kd1*q1_e_diff(i);
    q2dot_c = kp2*q2_diff(i) + kd2*q2_e_diff(i);   
    %Torque is based on counter-gravity torque + control signal
    tau = [ (g*m2*(rC2*cos(q(1) + q(2)) + L1*cos(q(1))) + g*m1*rC1*cos(q(1))) + q1dot_c;
        (g*m2*rC2*cos(q(1) + q(2))) + q2dot_c];
    
    [t,y] = ode45(@(t,y) runrobot(t,y,tau), [0, dt_PID], [q(1), q(2), qdot(1), qdot(2)]);
    Ly = length(y(:,1));
    q(1) = y(Ly,1);
    q(2) = y(Ly,2);
    qdot(1) = y(Ly,3);
    qdot(2) = y(Ly,4);
    
    % Limited by the physical limit
    [q(1),q(2),qdot(1),qdot(2)] = Task3_angle_limit(q(1),q(2),qdot(1),qdot(2));    
    q1s_b(i) = q(1)*180/pi;
    q2s_b(i) = q(2)*180/pi;
    qdot1s_b(i) = qdot(1)*180/pi;
    qdot2s_b(i) = qdot(2)*180/pi;
    torque(i,:) = tau;    
    i = i+1;
end
toc;
fprintf('finished Traj B simulation\n')