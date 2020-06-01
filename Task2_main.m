%% Robotic Systems Assignment 4
% Author: Jonathan Wong
% Name: Task2_main.m
% Purpose: All functions and implementations will be called from here to
%          execute task 2 questions 2.1, 2.2, 2.3.
clc
clear all
close all
dt = 0.01; 
tf = 5; 

% Initialise the robot to the initial position and velocity
q(1)= 0/180*pi; % Converting degree to radian
q(2) = 0; 
qdot(1) = 0;
qdot(2) = 0;
i = 1;
q1 = q(1);
q2 = q(2);
q1_d = qdot(1);
q2_d = qdot(2);
% initialise PID variables for loop
cumulativeE1 = 0;
cumulativeE2 = 0;

%% Variable Declaration
L_1 = 1; % length of link 1 /m
L_2 = 0.6; % length of link 2 /m
r_c1 = L_1/2; % distance from origin of link 1 to CoM for link 1 /m
r_c2 = L_2/2; % distance from origin of link 2 to CoM for link 2 /m
m_1 = 2; % mass of link 1
m_2 = 1; % mass of link 2
I_1 = [0 0 0;
       0 0 0;
       0 0 0.5];  % angular moment of inertia for link 1 about CoM
I_2 = [0 0 0;
       0 0 0;
       0 0 0.3];  % angular moment of inertia for link 2 about CoM
g = 9.81;

%% Trajectory Generation
segmentCoeff_q1 = TrajGen_each_seg([30,0], [-15,0], [0 5]);
segmentCoeff_q2 = TrajGen_each_seg([75,0], [45,0], [0 5]);
q1_d_out = 0;
q2_d_out = 0;


for time=0:dt:tf
   
    %% Find torque
    tic
    Jv_1_transposed = [-sin(q1)/2,cos(q1)/2,0;
                   0,0,0];
    Jv_2_transposed = [(3*cos(q1)*cos(q2 + pi/2))/10 - sin(q1) - (3*sin(q1)*sin(q2 + pi/2))/10, cos(q1) + (3*cos(q1)*sin(q2 + pi/2))/10 + (3*cos(q2 + pi/2)*sin(q1))/10, 0;
                   (3*cos(q1)*cos(q2 + pi/2))/10 - (3*sin(q1)*sin(q2 + pi/2))/10,(3*cos(q1)*sin(q2 + pi/2))/10 + (3*cos(q2 + pi/2)*sin(q1))/10, 0];
    tau1 = Jv_1_transposed*[0;(m_1)*g;0];
    tau2 = Jv_2_transposed*[0;(m_2)*g;0]; % This is the command torque, for now it is set to zero
    tau = tau1+tau2 % gravity compensation
    % add in the input from controller
    % tau = tau+[Qc1;Qc2];
    q(1)=q1;
    q(2)=q2;
    toc
    tic
    [t,y] = ode45(@(t,y) Task2_dynamics(t,y,tau), [0, dt], [q(1), q(2), qdot(1), qdot(2)]);
    toc
    Ly = length(y(:,1));
    q(1) = y(Ly,1);
    q(2) = y(Ly,2);
    qdot(1) = y(Ly,3);
    qdot(2) = y(Ly,4);
    q1_out = q(1);
    q2_out = q(2);
    q1_d_out = qdot(1);
    q2_d_out = qdot(2);
    
    %%% Storing q and qdot values into an array
    q1s(i) = q(1)*180/pi;
    q2s(i) = q(2)*180/pi;
    qdot1s(i) = qdot(1)*180/pi;
    qdot2s(i) = qdot(2)*180/pi;
    i = i+1;

end

time = 0:dt:tf;
figure (1)
plot(time,q1s,'-o', time,q2s,'-o')
legend('q1','q2')
k = 1;
figure (2)
for i = 1:(length(q1s)/50)
    plot_robot(q1s(k),q2s(k))
    k = i*50;
end