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
end
toc;
fprintf('finished calculating reference in task space\n')
%% Define the initial position in joint space
% Initialise the robot to the initial position and velocity
% q for control loop
[q1_ref_init, q2_ref_init] = two_arm_IK(x_ref(1),y_ref(1),false);
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
fprintf('finished simulation\n')
