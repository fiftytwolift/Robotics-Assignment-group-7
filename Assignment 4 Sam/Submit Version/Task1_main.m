close all;
clear all;
%% Run the simulation and dynamics of the robot
Task1_dynamics

%% Task 1.1 All variables output

A = simplify(A)
B = simplify(B)
C = simplify(C)
b = simplify(b)
G = simplify(G)
    
%% Task 1.1 free swing
dt = 0.01;
tf = 5;

% Initialise the robot to the initial position and velocity
q(1)= 0.0; % Converting degree to radian
q(2) = 0.0;
qdot(1) = 0.0;
qdot(2) = 0.0;
i = 1;

tau = [0;0];  % Zero torque

for time=0:dt:tf
    
    
    [t,y] = ode45(@(t,y) runrobot(t,y,tau), [0, dt], [q(1), q(2), qdot(1), qdot(2)]);
    Ly = length(y(:,1));
    q(1) = y(Ly,1);
    q(2) = y(Ly,2);
    qdot(1) = y(Ly,3);
    qdot(2) = y(Ly,4);
    
    %%% Storing q and qdot values into an angle array
    q1s(i) = q(1)*180/pi;
    q2s(i) = q(2)*180/pi;
    qdot1s(i) = qdot(1)*180/pi;
    qdot2s(i) = qdot(2)*180/pi;
    i = i+1;
    
end
figure();
time = 0:dt:tf;
plot(time,q1s,'-o', time,q2s,'-o')
title('Free drop')
xlabel('time / s')
ylabel('Joint angle / degree')
legend('q1','q2')

%% Task 1.2 Gravity compensation
clear q qdot i q1s q2s qdot1s qdot2s;
% Re-initialise the robot to the initial position and velocity
q(1)= 0.0; % Converting degree to radian
q(2) = 0.0;
qdot(1) = 0.0;
qdot(2) = 0.0;
i = 1;
tau = [ g*m2*(rC2 + L1) + g*m1*rC1; g*m2*rC2];
for time=0:dt:tf    
    [t,y] = ode45(@(t,y) runrobot(t,y,tau), [0, dt], [q(1), q(2), qdot(1), qdot(2)]);
    Ly = length(y(:,1));
    q(1) = y(Ly,1);
    q(2) = y(Ly,2);
    qdot(1) = y(Ly,3);
    qdot(2) = y(Ly,4);
    
    %%% Storing q and qdot values into an angle array
    q1s(i) = q(1)*180/pi;
    q2s(i) = q(2)*180/pi;
    qdot1s(i) = qdot(1)*180/pi;
    qdot2s(i) = qdot(2)*180/pi;
    i = i+1;
    
end
figure();
time = 0:dt:tf;
plot(time,q1s,'-o', time,q2s,'-o')
axis([0 tf -10 10])
xlabel('time / s')
ylabel('Joint angle / degree')
title('Gravity compensated')
legend('q1','q2')

%% Task 1.2 Gravity compensation but insufficient
clear q qdot i q1s q2s qdot1s qdot2s;
% Re-initialise the robot to the initial position and velocity
q(1)= 0.0; % Converting degree to radian
q(2) = 0.0;
qdot(1) = 0.0;
qdot(2) = 0.0;
i = 1;
m2 = 0.995;
tau = [ g*m2*(rC2 + L1) + g*m1*rC1; g*m2*rC2];
for time=0:dt:tf    
    [t,y] = ode45(@(t,y) runrobot(t,y,tau), [0, dt], [q(1), q(2), qdot(1), qdot(2)]);
    Ly = length(y(:,1));
    q(1) = y(Ly,1);
    q(2) = y(Ly,2);
    qdot(1) = y(Ly,3);
    qdot(2) = y(Ly,4);
    
    %%% Storing q and qdot values into an angle array
    q1s(i) = q(1)*180/pi;
    q2s(i) = q(2)*180/pi;
    qdot1s(i) = qdot(1)*180/pi;
    qdot2s(i) = qdot(2)*180/pi;
    i = i+1;
    
end
figure();
time = 0:dt:tf;
plot(time,q1s,'-o', time,q2s,'-o')
axis([0 tf -10 10])
title('Insufficient Gravity compensated')
xlabel('time / s')
ylabel('Joint angle / degree')
legend('q1','q2')
