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
kp_x = 1;
kp_y = 1;
Kp = diag([kp_x kp_y]);

kp1 = 1250;
kp2 = 380;
kd1 = 0.05;
kd2 = 0.02;

% coefficient for the avoidance part
OutBound = -999;
interact_limit = 0.2;
force_field_limit = 0.010;
force_field_constant = 8;
num_trial = 100;
eps = 1e-20;
final_e = [];
stop_limit = 0.02;

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