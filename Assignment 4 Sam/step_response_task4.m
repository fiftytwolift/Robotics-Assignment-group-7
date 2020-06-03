clear all
close all
%% Initiate the variables for the trajectory
x_init = 0.71;
y_init = 1.08;
x_final = 0.81;
y_final = 0.98;

xdot_init = 0;
ydot_init = 0;
xdot_final = 0;
ydot_final = 0;

tfinal = 0.1;
dt = 0.01;
dt_PID = 0.001;

kp_x = 50;
kp_y = 50;
Kp = diag([kp_x kp_y]);

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
syms q1 q2
J_v = [-L2*sin(q1+q2) - L1*sin(q1), -L2*sin(q1+q2);...
    L2*cos(q1+q2) + L1*cos(q1), L2*cos(q1+q2);...
    0,0];
J_v_xy_inv = inv(J_v(1:2,1:2));

%% Generate the points on the trajectory
tic;
syms q1 q2
qdot_buffer =[];
time_control=0:dt:tfinal;
for time_index = 1: length(time_control)
    x_ref(time_index) = x_final;
    y_ref(time_index) = y_final;
    xdot_ref(time_index) = xdot_init;
    ydot_ref(time_index) = ydot_init;
end
toc;
fprintf('finished calculating reference in task space\n')
%% Define the initial position in joint space
% Initialise the robot to the initial position and velocity
% q for control loop
[q1_ref_init, q2_ref_init] = two_arm_IK(x_init,y_init,false);
[q(1),q(2),qdot(1),qdot(2)] = ...
    Task3_angle_limit(q1_ref_init/180*pi,q2_ref_init/180*pi,0,0);

%q for output
q1s(1) = q(1)*180/pi; % Convert back to angle
q2s(1) = q(2)*180/pi;
qdot1s(1) = 0;
qdot2s(1) = 0;

i = 1;
tic;
angular_velocity_c = [0;0];
angular_velocity_c_old_buffer = [0;0];
linear_velocity_c = [0;0];

for time=0:dt_PID:tfinal
    if mod(i,floor(dt/dt_PID)) == 1
        % Points of end-effector
        mid_pt = [L1*cos(q(1)),L1*sin(q(1))];
        end_effector = mid_pt+[L2*cos(q(1)+q(2)),L2*sin(q(1)+q(2))];
        
        % Error term
        x_e = x_ref(ceil(i/dt*dt_PID)) - end_effector(1);
        y_e = y_ref(ceil(i/dt*dt_PID)) - end_effector(2);

        % velocity tracting signal
        xy_vel_track = [xdot_ref(ceil(i/dt*dt_PID));ydot_ref(ceil(i/dt*dt_PID))];
        % Control signal
        linear_velocity_c = xy_vel_track + Kp*[x_e;y_e];
    
        % Before overwriting the control signal, save the old one
        angular_velocity_c_old_buffer = angular_velocity_c;
        % converting back to angular velocity control signal 
        angular_velocity_c = round(double(subs(J_v_xy_inv,[q1 q2],[q(1) q(2)])),3)...
            *linear_velocity_c;
    end
    
    % Error term in angular velocity
    q1_diff(i) = angular_velocity_c(1) - qdot(1);
    q2_diff(i) = angular_velocity_c(2) - qdot(2);
    
    % Error term in angular accel
    q1_e_diff(i) = (angular_velocity_c(1) - angular_velocity_c_old_buffer(1))/dt-(q1_diff(i)-q1_diff(max(i-1,1)))/dt_PID;
    q2_e_diff(i) = (angular_velocity_c(2) - angular_velocity_c_old_buffer(2))/dt-(q2_diff(i)-q2_diff(max(i-1,1)))/dt_PID;
    
    % control signal for each joint
    q1dot_c = kp1*q1_diff(i) + kd1*q1_e_diff(i);
    q2dot_c = kp2*q2_diff(i) + kd2*q2_e_diff(i); 
    
    %Torque applied
    tau = [ g*m2*(rC2*cos(q(1) + q(2)) + L1*cos(q(1))) + g*m1*rC1*cos(q(1)) + q1dot_c;
        g*m2*rC2*cos(q(1) + q(2)) + q2dot_c];
    
    
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
%% Plotting the trajectory
i = 1;
for t = 0:dt_PID:tfinal
    end_effector_x(i) = [L1*cosd(q1s(i)) + L2*cosd(q1s(i)+q2s(i))];
    end_effector_y(i) = [L1*sind(q1s(i)) + L2*sind(q1s(i)+q2s(i))];
    i = i+1;
end
figure('Name','Task 4.1')
subplot(1,2,1)
title('Task space position control step response x vs t', 'FontSize',15)
hold on
plot([0:dt_PID:tfinal],end_effector_x,'r')
stairs([0:dt:tfinal],x_ref,'b')
xlabel('Time/s', 'FontSize', 13)
ylabel('x / m', 'FontSize', 13)
legend('real trajectory','reference trajectory', 'FontSize', 13)

subplot(1,2,2)
title('Task space position control step response y vs t', 'FontSize', 15)
hold on
plot([0:dt_PID:tfinal],end_effector_y,'r')
stairs([0:dt:tfinal],y_ref,'b')
xlabel('Time/s', 'FontSize', 13)
ylabel('y / m', 'FontSize', 13)
legend('real trajectory','reference trajectory', 'FontSize', 13)