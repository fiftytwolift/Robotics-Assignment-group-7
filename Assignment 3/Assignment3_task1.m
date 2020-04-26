%% set up variables
close all
clear all
tfinal = [3,8];

%% calculate the segments coefficents for x and y
x_coeff = TrajGen([2,0],[14,0],[5,0.25],tfinal);
y_coeff = TrajGen([5,0],[7,0],[8,0.25],tfinal);

%% get the displacement and velocity in both x and y
x_displacement = plot_displacement(x_coeff,tfinal,true);
y_displacement = plot_displacement(y_coeff,tfinal,true);
plot_velocity(x_coeff,tfinal);
plot_velocity(y_coeff,tfinal);


%% plot the final trajectory
figure()
hold on
plot(x_displacement,y_displacement)
pbaspect([14 8.5 1])
axis([0 14 0 8.5])
xlabel('x / m')
ylabel('y / m')