close all
clear all
%% Initiate the variables for the trajectory
q1_init = 30;
q2_init = 75;
q1_final = -15;
q2_final = 45;

% q1_init = 0;
% q2_init = 0;
% q1_final = 0;
% q2_final = 0;

q1dot_init = 0;
q2dot_init = 0;
q1dot_final = 0;
q2dot_final = 0;
tfinal = 5;
dt = 0.01;
dt_PID = 0.002;
kp = 5000;
ki = 0;
kd = 1;

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
%     q1_ref(time_index) = 1;
%     q2_ref(time_index) = 1;
%     q1dot_ref(time_index) = 0;
%     q2dot_ref(time_index) = 0;
end

%% PID controller 
% Initialise the robot to the initial position and velocity
% q for control loop
q(1)= q1_init/180*pi; % Converting degree to radian
q(2) = q2_init/180*pi;
qdot(1) = q1dot_init/180*pi;
qdot(2) = q2dot_init/180*pi;
%q for robot sim
q_robot(1)= q1_init/180*pi; % Converting degree to radian
q_robot(2) = q2_init/180*pi;
qdot_robot(1) = q1dot_init/180*pi;
qdot_robot(2) = q2dot_init/180*pi;
%q for output
q1s(1) = q_robot(1)*180/pi;
q2s(1) = q_robot(2)*180/pi;
qdot1s(1) = qdot_robot(1)*180/pi;
qdot2s(1) = qdot_robot(2)*180/pi;
q1_e_integrate = 0;
q2_e_integrate = 0;
i = 1;
PID_i = 1;
for time=0:dt_PID:tfinal
    % Error term
    q1_e = q1_ref(floor(i/5)+1)/180*pi - q(1);
    q2_e = q2_ref(floor(i/5)+1)/180*pi - q(2);
    
    % Integral term
    q1_e_integrate = dt_PID*q1_e + q1_e_integrate;
    q2_e_integrate = dt_PID*q2_e + q2_e_integrate;
    
    % Differential term
    q1_e_diff = q1dot_ref(floor(i/5)+1) - qdot(1);
    q2_e_diff = q2dot_ref(floor(i/5)+1) - qdot(2);
    
    q1dot_c = q1dot_ref(floor(i/5)+1)/180*pi + kp*q1_e + ki*q1_e_integrate + kd*q1_e_diff;
    q2dot_c = q2dot_ref(floor(i/5)+1)/180*pi + kp*q2_e + ki*q2_e_integrate + kd*q2_e_diff;
    
    
    tau = [ g*m2*(rC2*cos(q(1) + q(2)) + L1*cos(q(1))) + g*m1*rC1*cos(q(1)) + q1dot_c;
        g*m2*rC2*cos(q(1) + q(2)) + q2dot_c];
    [t,y] = ode45(@(t,y) runrobot(t,y,tau), [0, dt_PID], [q(1), q(2), qdot(1), qdot(2)]);
    Ly = length(y(:,1));
    q(1) = y(Ly,1);
    q(2) = y(Ly,2);
    qdot(1) = y(Ly,3);
    qdot(2) = y(Ly,4);
    %   robot update
    if(mod(i,5) ==0)
        [t_robot,y_robot] = ode45(@(t_robot,y_robot) runrobot(t_robot,y_robot,tau), [0, dt], [q_robot(1), q_robot(2), qdot_robot(1), qdot_robot(2)]);
        Ly_robot = length(y_robot(:,1));
        q_robot(1) = y_robot(Ly_robot,1);
        q_robot(2) = y_robot(Ly_robot,2);
        qdot_robot(1) = y_robot(Ly_robot,3);
        qdot_robot(2) = y_robot(Ly_robot,4);
        %update the control loop
        q(1) = q_robot(1);
        q(2) = q_robot(2);
        qdot(1) = qdot_robot(1);
        qdot(2) = qdot_robot(2);
        %%% Storing q and qdot values into an angle array, also saving the
        %%% torque
        q1s(i/5+1) = q_robot(1)*180/pi;
        q2s(i/5+1) = q_robot(2)*180/pi;
        qdot1s(i/5+1) = qdot_robot(1)*180/pi;
        qdot2s(i/5+1) = qdot_robot(2)*180/pi;
        torque(i/5+1,:) = tau;
    end    
    i = i+1;
end

%% Plotting the angles and angular velocities
figure()
subplot(2,2,1)
hold on
plot([0:dt:tfinal],q1s,'r')
plot([0:dt:tfinal],q1_ref,'b')
xlabel('Time/s')
ylabel('Angle/Degree')
title('q1 vs time ')

subplot(2,2,2)
hold on
plot([0:dt:tfinal],qdot1s,'r')
plot([0:dt:tfinal],q1dot_ref,'b')
xlabel('Time/s')
ylabel('Angular velocity /Deg per sec')
title('q1dot vs time ')

subplot(2,2,3)
hold on
plot([0:dt:tfinal],q2s,'r')
plot([0:dt:tfinal],q2_ref,'b')
xlabel('Time/s')
ylabel('Angle/Degree')
title('q2 vs time')

subplot(2,2,4)
hold on
plot([0:dt:tfinal],qdot2s,'r')
plot([0:dt:tfinal],q2dot_ref,'b')
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
