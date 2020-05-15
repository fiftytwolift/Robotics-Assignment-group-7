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
kd_x = 100;
kd_y = 100;
kp = 2;
OutBound = -999;
K = diag([kd_x kd_y]);
force_field_limit = 0.1;
force_field_constant = 4;
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
% Looping the simulation to get multiple test
for j = 1:25
    %% Define the initial position in joint space
    %% PID joint space controller
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
    PID_i = 1;
    tic;
    % Before the simulation, create an object
    sensor_t5(1,OutBound,OutBound);
    obj_i = 1;
    estimate_obj_xpos = [];
    estimate_obj_ypos = [];
    final_e = [];
    min_d(j) = 1.000;
    last_dist = 1000;
    avoid_xdot = 0;
    avoid_ydot = 0;
    percentage_progress = -20;
    for time=0:dt_PID:tfinal
        % Update the sensor value
        % Points of end-effector
        mid_pt = [L1*cos(q(1)),L1*sin(q(1))];
        end_effector = mid_pt+[L2*cos(q(1)+q(2)),L2*sin(q(1)+q(2))];
        [d,th] = sensor_t5(0,end_effector(1),end_effector(2));
        if d<0.350
            estimate_obj_xpos(obj_i) = end_effector(1) + d*cos(th);
            estimate_obj_ypos(obj_i) = end_effector(2) + d*sin(th);
            obj_i = obj_i + 1;
            
            % Interaction
            movement_dir = atan2(ydot_ref(floor(i/dt*dt_PID)+1),xdot_ref(floor(i/dt*dt_PID)+1));
            if abs(movement_dir-th) < pi/2 && d<last_dist
                % Calculate the current movement speed for reference
                move_speed = (ydot_ref(floor(i/dt*dt_PID)+1)^2 + xdot_ref(floor(i/dt*dt_PID)+1)^2)^0.5;
                %             fprintf("Avoid maneuver\n")
                % IF we are above the obstacle
                if (movement_dir+pi/6<th)
                    avoid_xdot = move_speed*force_field_constant*force_field_limit/...
                        (d+force_field_limit)*sin(th);
                    avoid_ydot = move_speed*force_field_constant*force_field_limit/...
                        (d+force_field_limit)*-cos(th);
                else
                    avoid_xdot = move_speed*force_field_constant*force_field_limit/...
                        (d+force_field_limit)*-sin(th);
                    avoid_ydot = move_speed*force_field_constant*force_field_limit/...
                        (d+force_field_limit)*cos(th);
                end
            else
                %             fprintf("Safe\n")
                avoid_xdot = 0;
                avoid_ydot = 0;
            end
            % Save the new distance
            last_dist = d;
            % Save the smallest d to check whether there is or not a collision
            if (d < min_d(j))
                min_d(j) = d;
            end
        end
        
        %Calculate the actual x,y velocity
        xy_vel_real = round(double(subs(J_v,[q1 q2],[q(1)*180/pi q(2)*180/pi])),3)*[qdot(1);qdot(2)];
        % Error term
        x_e = x_ref(floor(i/dt*dt_PID)+1) - end_effector(1);
        y_e = y_ref(floor(i/dt*dt_PID)+1) - end_effector(2);
        
        xdot_e = xdot_ref(floor(i/dt*dt_PID)+1) - xy_vel_real(1);
        ydot_e = ydot_ref(floor(i/dt*dt_PID)+1) - xy_vel_real(2);
        
        % Control signal
        % Avoidance mode
        if abs(avoid_xdot) > 0
            avoid_xdot;
            linear_velocity_c = K*[xdot_e+avoid_xdot;ydot_e+avoid_ydot];
            % PD mode
        else
            linear_velocity_c = K*[xdot_e+kp*x_e;ydot_e+kp*y_e];
        end
        angular_velocity_c = round(double(subs(J_v_xy_inv,[q1 q2],[q(1)*180/pi q(2)*180/pi])),3)...
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
        % Limited by the physical limit
        [q(1),q(2),qdot(1),qdot(2)] = Task3_angle_limit(q(1),q(2),qdot(1),qdot(2));
        
        q1s(i) = q(1)*180/pi;
        q2s(i) = q(2)*180/pi;
        qdot1s(i) = qdot(1)*180/pi;
        qdot2s(i) = qdot(2)*180/pi;
        torque(i,:) = tau;
        i = i+1;
        %Progress logger
        if mod(time/dt_PID,(tfinal/dt_PID)/5)==0
            percentage_progress = percentage_progress + 20;
            fprintf("The percentage progress is %2.0f\n",percentage_progress);
        end
    end
    min_d(j)
    final_e = [final_e;x_e y_e]
    toc;
    fprintf('finished simulation\n')
    %% Plotting the robot
    subplot(5,5,j)
    for t = 0:tfinal/30:tfinal
        extractedQ1 = q1s(floor(t/dt_PID)+1);
        extractedQ2 = q2s(floor(t/dt_PID)+1);
        plot_robot(extractedQ1,extractedQ2);
    end
    % Also, plot the avoided obstacle if it detected one
    if length(estimate_obj_xpos)>=1
        scatter(estimate_obj_xpos,estimate_obj_ypos);
    end
    title('Trajectory A in taskspace velocity control')
    xlabel('x / m')
    ylabel('y / m')
end
% %% Plotting the real angular velocities
% figure()
%
% subplot(1,2,1)
% hold on
% plot([0:dt_PID:tfinal],qdot1s,'r')
% xlabel('Time/s')
% ylabel('Angular velocity /Deg per sec')
% title('q1dot vs time ')
%
% subplot(1,2,2)
% hold on
% plot([0:dt_PID:tfinal],qdot2s,'r')
% xlabel('Time/s')
% ylabel('Angular velocity /Deg per sec')
% title('q2dot vs time')
%
% %% Plotting the torques
% figure()
% hold on
% plot([0:dt_PID:tfinal],torque(:,1),'r')
% plot([0:dt_PID:tfinal],torque(:,2),'b')
% xlabel('Time/s')
% ylabel('Torque/Nm')
% legend('Torque for joint 1','Torque for joint 2')
% title('Torque for joint 1 and joint 2')




