close all
clear all
%% Run the simulation and dynamics of the robot
Task2_dynamics;

%% Task 2.1 Plotting the angles and angular velocities
figure('Name','Task 2.1')
subplot(2,2,1)
hold on
plot([0:dt_PID:tfinal],q1s,'r')
plot([0:dt:tfinal],q1_ref,'b')
xlabel('Time/s')
ylabel('Angle/Degree')
legend('real trajectory','reference trajectory','FontSize', 13)
title('Task 2.1 q1 vs time ','FontSize', 15)

subplot(2,2,2)
hold on
plot([0:dt_PID:tfinal],qdot1s,'r')
stairs([0:dt:tfinal],q1dot_ref,'b')
xlabel('Time/s')
ylabel('Angular velocity /Deg per sec')
legend('real velocity','reference velocity','FontSize', 13)
title('Task 2.1 q1dot vs time ','FontSize', 15)

subplot(2,2,3)
hold on
plot([0:dt_PID:tfinal],q2s,'r')
stairs([0:dt:tfinal],q2_ref,'b')
xlabel('Time/s')
ylabel('Angle/Degree')
legend('real trajectory','reference trajectory','FontSize', 13)
title('Task 2.1 q2 vs time','FontSize', 15)

subplot(2,2,4)
hold on
plot([0:dt_PID:tfinal],qdot2s,'r')
stairs([0:dt:tfinal],q2dot_ref,'b')
xlabel('Time/s')
ylabel('Angular velocity /Deg per sec')
legend('real velocity','reference velocity','FontSize', 13)
title('Task 2.1 q2dot vs time','FontSize', 15)

%% Task 2.2 Plotting the torques
figure('Name','Task 2.2')
hold on
plot([0:dt_PID:tfinal],torque(:,1),'r')
plot([0:dt_PID:tfinal],torque(:,2),'b')
xlabel('Time/s', 'FontSize', 13)
ylabel('Torque/Nm', 'FontSize', 13)
legend('Torque for joint 1','Torque for joint 2', 'FontSize', 13)
title('Task 2.2 Torque for joint 1 and joint 2','FontSize', 15)

%% Task 2.3 Plotting the robot
figure('Name','Task 2.3')
for t = 0:tfinal/10:tfinal
    extractedQ1 = q1s(floor(t/dt_PID)+1);
    extractedQ2 = q2s(floor(t/dt_PID)+1);
    plot_robot(extractedQ1,extractedQ2);
end
title('Task 2.3 Jointspace velocity control trajectory');
xlabel('x / m')
ylabel('y / m')