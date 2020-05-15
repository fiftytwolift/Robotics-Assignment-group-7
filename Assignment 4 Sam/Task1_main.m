close all;
clear all;

% robot simulation part
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
%% first for free swing
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
axis([0 5 -180 180])
legend('q1','q2')

%% Second part, Gravity compensation
clear q qdot i q1s q2s qdot1s qdot2s;
% Re-initialise the robot to the initial position and velocity
q(1)= 0.0; % Converting degree to radian
q(2) = 0.0;
qdot(1) = 0.0;
qdot(2) = 0.0;
i = 1;
for time=0:dt:tf
    
    tau = [ g*m2*(rC2*cos(q(1) + q(2)) + L1*cos(q(1))) + g*m1*rC1*cos(q(1)); g*m2*rC2*cos(q(1) + q(2))];
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
title('Gravity compensated')
axis([0 5 -180 180])
legend('q1','q2')
