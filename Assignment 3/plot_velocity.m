function velocity = plot_velocity(segmentCoeff, tfinal)

%% checking the size and validity of the input
if (length(segmentCoeff)/4 ~= length(tfinal))
    warning('The input is not valid')
    return;
end
%% plot the cubic spline
% get the array for t
t = 0;
delta = 0.01;
for i = 1:length(tfinal)
    t = [t (t(end)+delta):delta:tfinal(i)];
end
% get the array for displacement
cur_segment = 1;
total_segment = length(segmentCoeff)/4;
for time_index = 1: length(t)
    if t(time_index) > tfinal(cur_segment) + delta
        cur_segment = cur_segment + 1;
    end
    velocity(time_index) = [zeros(1,(cur_segment-1)*4) 3*t(time_index)^2 2*t(time_index)^1 1 0 zeros(1,(total_segment-cur_segment)*4)]*segmentCoeff;
end
figure()
plot(t,velocity);
title('velocity vs time')
xlabel('time/s')
ylabel('velocity / ms^-1')
