close all
clear all
%% Run the simulation and dynamics of the robot
Task5_dynamcis;

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
min_d = 1000;
obj_i = 1;
estimate_obj_xpos = [];
estimate_obj_ypos = [];
avoid_xdot = 0;
avoid_ydot = 0;
percentage_progress = -20;
angular_velocity_c = [0;0];
angular_velocity_c_old_buffer = [0;0];
linear_velocity_c = [0;0];
for time=0:dt_PID:tfinal
    if mod(i,floor(dt/dt_PID)) == 1
        
        % Update the sensor value
        % Points of end-effector
        mid_pt = [L1*cos(q(1)),L1*sin(q(1))];
        end_effector = mid_pt+[L2*cos(q(1)+q(2)),L2*sin(q(1)+q(2))];
        [d,th] = sensor_t5(0,end_effector(1),end_effector(2));
        if d<interact_limit
            estimate_obj_xpos(obj_i) = end_effector(1) + d*cos(th);
            estimate_obj_ypos(obj_i) = end_effector(2) + d*sin(th);
            obj_i = obj_i + 1;
            
            % Interaction
            movement_dir = atan2(ydot_ref(floor(i/dt*dt_PID)+1),xdot_ref(floor(i/dt*dt_PID)+1));
            % Before the obstacle
            if abs(movement_dir-th) < pi/2
                % Calculate the current movement speed for reference
                move_speed = (ydot_ref(floor(i/dt*dt_PID)+1)^2 + xdot_ref(floor(i/dt*dt_PID)+1)^2)^0.5;
                % IF above the obstacle
                if (movement_dir+pi/6<th)
                    % fprintf("Moving towards in below position\n")
                    avoid_xdot = move_speed*force_field_constant*force_field_limit/...
                        (d+force_field_limit)*sin(th);
                    avoid_ydot = move_speed*force_field_constant*force_field_limit/...
                        (d+force_field_limit)*-cos(th);
                    from_above = true;
                    % IF below the obstacle
                else
                    % fprintf("Moving towards in above position\n")
                    avoid_xdot = move_speed*force_field_constant*force_field_limit/...
                        (d+force_field_limit)*-sin(th);
                    avoid_ydot = move_speed*force_field_constant*force_field_limit/...
                        (d+force_field_limit)*cos(th);
                    from_above = false;
                    
                end
                % After the obstacle
            elseif  abs(movement_dir-th)>pi/2
                % IF above the obstacle
                if from_above == true
                    % fprintf("Moving pass in above position\n")
                    avoid_xdot = move_speed*force_field_constant*force_field_limit/...
                        (d+force_field_limit)*sin(th);
                    avoid_ydot = move_speed*force_field_constant*force_field_limit/...
                        (d+force_field_limit)*-cos(th);
                    % IF below the obstacle
                else
                    % fprintf("Moving pass in below position\n")
                    avoid_xdot = move_speed*force_field_constant*force_field_limit/...
                        (d+force_field_limit)*-sin(th);
                    avoid_ydot = move_speed*force_field_constant*force_field_limit/...
                        (d+force_field_limit)*cos(th);
                end
            end
        else
            % fprintf("Safe\n")
            avoid_xdot = 0;
            avoid_ydot = 0;
        end
        
        % Save the smallest d to check whether there is or not a collision
        if (d < min_d)
            min_d = d;
            % Print the collison condition
            if d == 0
                fprintf("theta is %2.4f at position %2.4f %2.4f\n",th,end_effector(1),end_effector(2))
            end
        end
        
        % Error term
        x_e = x_ref(ceil(i/dt*dt_PID)) - end_effector(1);
        y_e = y_ref(ceil(i/dt*dt_PID)) - end_effector(2);
        
        % velocity tracting signal
        xy_vel_track = [xdot_ref(ceil(i/dt*dt_PID));ydot_ref(ceil(i/dt*dt_PID))];
        
        % Control signal
        if abs(avoid_xdot) > eps
            % Avoidance mode
            linear_velocity_c = xy_vel_track + [avoid_xdot;avoid_ydot];
        else
            % P control
            linear_velocity_c = xy_vel_track + Kp*[x_e;y_e];
        end
        angular_velocity_c = round(double(subs(J_v_xy_inv,[q1 q2],[q(1)*180/pi q(2)*180/pi])),3)...
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
    %Progress logger
    if mod(time/dt_PID,(tfinal/dt_PID)/5)==0
        percentage_progress = percentage_progress + 20;
        fprintf("The percentage progress is %2.0f\n",percentage_progress);
    end
end
% if we want to see the result on the fly, don't suppress the output
min_d;
final_e = [x_e y_e];
toc;
fprintf('finished simulation\n')
%% Plotting the robot
pbaspect([1.5 1.5 1])
axis([0 1.5 -0.2 1.3])
for t = 0:tfinal/10:tfinal
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
