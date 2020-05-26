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
dt_PID = 0.002;
kp_x = 100;
kp_y = 100;
% kp_z = 1;
% kp_x_or = 1;
% kp_y_or = 1;
% kp_z_or = 1;
% K = diag([kp_x kp_y kp_z kp_x_or kp_y_or kp_z_or]);
K = diag([kp_x kp_y]);

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
J_v = [-L2*sind(q1+q2) - L1*sind(q1), -L2*sind(q1+q2);...
         L2*cosd(q1+q2) + L1*cosd(q1), L2*cosd(q1+q2);...
         0,0];
J_w = [0, 0;...
       0, 0;...
       1, 1];
J_v_xy_inv = inv(J_v(1:2,1:2));
R0_E = [cos(q1 + q2), -sin(q1 + q2), 0;...
        sin(q1 + q2),  cos(q1 + q2), 0;
        0           ,  0           , 1];
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
%% PID joint space controller
% Initialise the robot to the initial position and velocity
% q for control loop
[q1_ref_init, q2_ref_init] = two_arm_IK(x_ref(1),y_ref(1),false);
[q(1),q(2),qdot(1),qdot(2)] = ...
    Task3_angle_limit(q1_ref_init/180*pi,q2_ref_init/180*pi,0,0);
%q for robot sim
q_robot(1)= q(1);
q_robot(2) = q(2);
qdot_robot(1) = 0;
qdot_robot(2) = 0;
%q for output
q1s(1) = q_robot(1)*180/pi; % Convert back to angle 
q2s(1) = q_robot(2)*180/pi;
qdot1s(1) = 0;
qdot2s(1) = 0;
x_e_integrate = 0;
y_e_integrate = 0;
i = 1;
PID_i = 1;
tic;
for time=0:dt_PID:tfinal
    %convert the current angle back to coordinates
    % points of joints at the robot
    mid_pt = [L1*cos(q(1)),L1*sin(q(1))];
    end_effector = mid_pt+[L2*cos(q(1)+q(2)),L2*sin(q(1)+q(2))];
    x_real = end_effector(1);
    y_real = end_effector(2);
    % Error term
    x_e = x_ref(floor(i/dt*dt_PID)+1) - x_real;
    y_e = y_ref(floor(i/dt*dt_PID)+1) - y_real;

    % Control signal
    linear_velocity_c = [xdot_ref(floor(i/dt*dt_PID)+1);...
                         ydot_ref(floor(i/dt*dt_PID)+1)] +...
                         K*[x_e;y_e];      
%     linear_velocity_c = K*[x_e;y_e];      
    angular_velocity_c = round(double(subs(J_v_xy_inv,[q1 q2],[q(1) q(2)])),3)...
                        *linear_velocity_c;
    %Torque applied
    tau = [ g*m2*(rC2*cos(q(1) + q(2)) + L1*cos(q(1))) + g*m1*rC1*cos(q(1)) + angular_velocity_c(1);
            g*m2*rC2*cos(q(1) + q(2)) + angular_velocity_c(2)];
    
    [t,y] = ode45(@(t,y) runrobot(t,y,tau), [0, dt_PID], [q(1), q(2), qdot(1), qdot(2)]);
    Ly = length(y(:,1));
    q(1) = y(Ly,1);
    q(2) = y(Ly,2);
    qdot(1) = y(Ly,3);
    qdot(2) = y(Ly,4);
    
    [q(1),q(2),qdot(1),qdot(2)] = Task3_angle_limit(q(1),q(2),qdot(1),qdot(2));
    
    %   robot update
    if(mod(i,5) ==0)
        [t_robot,y_robot] = ode45(@(t_robot,y_robot) runrobot(t_robot,y_robot,tau), [0, dt], [q_robot(1), q_robot(2), qdot_robot(1), qdot_robot(2)]);
        Ly_robot = length(y_robot(:,1));
        q_robot(1) = y_robot(Ly_robot,1);
        q_robot(2) = y_robot(Ly_robot,2);
        qdot_robot(1) = y_robot(Ly_robot,3);
        qdot_robot(2) = y_robot(Ly_robot,4);
        [q_robot(1),q_robot(2),qdot_robot(1),qdot_robot(2)] = ...
        Task3_angle_limit(q_robot(1),q_robot(2),qdot_robot(1),qdot_robot(2));
        %update the control loop
        q(1) = q_robot(1);
        q(2) = q_robot(2);
        qdot(1) = qdot_robot(1);
        qdot(2) = qdot_robot(2);
        %%% Storing q and qdot values into an angle array, also saving the
        %%% torque
        q1s(i/5+1) = q(1)*180/pi;
        q2s(i/5+1) = q(2)*180/pi;
        qdot1s(i/5+1) = qdot(1)*180/pi;
        qdot2s(i/5+1) = qdot(2)*180/pi;
        torque(i/5+1,:) = tau;
    end
    i = i+1;
    
end
toc;
fprintf('finished simulation\n')
%% Plotting the real angular velocities

figure()

subplot(1,2,1)
hold on
plot([0:dt:tfinal],qdot1s,'r')
xlabel('Time/s')
ylabel('Angular velocity /Deg per sec')
title('q1dot vs time ')

subplot(1,2,2)
hold on
plot([0:dt:tfinal],qdot2s,'r')
xlabel('Time/s')
ylabel('Angular velocity /Deg per sec')
title('q2dot vs time')

%% Plotting the torques
figure()
hold on
plot([0:dt:tfinal],torque(:,1),'r')
plot([0:dt:tfinal],torque(:,2),'b')
xlabel('Time/s')
ylabel('Torque/Nm')
legend('Torque for joint 1','Torque for joint 2')
title('Torque for joint 1 and joint 2')

%% Plotting the robot
figure()
for t = 0:0.5:tfinal
    extractedQ1 = q1s(floor(t/dt)+1);
    extractedQ2 = q2s(floor(t/dt)+1);
    plot_robot(extractedQ1,extractedQ2);
end
