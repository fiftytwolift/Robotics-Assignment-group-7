close all
clear all
%% Run the simulation and dynamics of the robot
Task4_dynamics; 

%% Task 4.1 Plotting the trajectory
i = 1;
for t = 0:dt_PID:tfinal
    end_effector_x(i) = [L1*cosd(q1s(i)) + L2*cosd(q1s(i)+q2s(i))];
    end_effector_y(i) = [L1*sind(q1s(i)) + L2*sind(q1s(i)+q2s(i))];
    i = i+1;
end
figure('Name','Task 4.1')
subplot(1,2,1)
title('Task 4.1 Trajectory A x vs t', 'FontSize',15)
hold on
plot([0:dt_PID:tfinal],end_effector_x,'r')
stairs([0:dt:tfinal],x_ref,'b')
xlabel('Time/s', 'FontSize', 13)
ylabel('x / m', 'FontSize', 13)
legend('real trajectory','reference trajectory', 'FontSize', 13)

subplot(1,2,2)
title('Task 4.1 Trajectory A y vs t', 'FontSize', 15)
hold on
plot([0:dt_PID:tfinal],end_effector_y,'r')
stairs([0:dt:tfinal],y_ref,'b')
xlabel('Time/s', 'FontSize', 13)
ylabel('y / m', 'FontSize', 13)
legend('real trajectory','reference trajectory', 'FontSize', 13)

%% Task 4.2 Plotting the robot
figure()
for t = 0:(tfinal/10):tfinal
    extractedQ1 = q1s(floor(t/dt_PID)+1);
    extractedQ2 = q2s(floor(t/dt_PID)+1);
    plot_robot(extractedQ1,extractedQ2);
end
title('Trajectory B in task space velocity control XY plot')
xlabel('x / m')
ylabel('y / m')

