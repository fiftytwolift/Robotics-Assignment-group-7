function segmentCoeff = TrajGen(init, final, via, tfinal)
%% checking the size and validity of the input
% assuming the via points are input in : 
% via = [via_pos(1) via_vel(1) via_pos(2) via_vel(2) ...]
if (length(init) ~= 2 ||  length(final) ~= 2 || mod(length(via),2) ~= 0)
    length(via)/2
    warning('The input is not valid')
    return;
end

if (length(tfinal) ~= length(via)/2+1)
    warning('The length of time array is not valid')
    return;
end
%% Setup the variables
init_pos = init(1);
init_vel = init(2);
final_pos = final(1);
final_vel = final(2);
init_time = 0;
final_time = tfinal(end);
for i = 0:length(via)/2-1
    via_time(i+1) = tfinal(i+1);
    via_pos(i+1) = via(i*2+1);
    via_vel(i+1) = via(i*2+2);
end
total_segment = length(via)/2 + 1;
%% set up a matrix to represent the cubic spline
%the equation are written in the form a*t^3 + b*t^2 + c*t + d = c
M_first_seg = [init_time^3 init_time^2 init_time^1 1 zeros(1,4*(total_segment-1));
               3*init_time^2 2*init_time 1 0 zeros(1,4*(total_segment-1));
               via_time(1)^3 via_time(1)^2 via_time(1)^1 1 zeros(1,4*(total_segment-1));
               3*via_time(1)^2 2*via_time(1) 1 0 zeros(1,4*(total_segment-1))];
M_via = []
for i = 1:total_segment-2
    M_via = [M_via;
            zeros(1,4*i) via_time(i)^3 via_time(i)^2 via_time(i)^1 1 zeros(1,4*(total_segment-1-i));
            zeros(1,4*i) 3*via_time(i)^2 2*via_time(i) 1 0 zeros(1,4*(total_segment-1-i));
            zeros(1,4*i) via_time(i+1)^3 via_time(i+1)^2 via_time(i+1)^1 1 zeros(1,4*(total_segment-1-i));
            zeros(1,4*i) 3*via_time(i+1)^2 2*via_time(i+1) 1 0 zeros(1,4*(total_segment-1-i))];
end
        
M_final_seg = [zeros(1,4*(total_segment-1)) via_time(end)^3 via_time(end)^2 via_time(end)^1 1;
               zeros(1,4*(total_segment-1)) 3*via_time(end)^2 2*via_time(end) 1 0;
               zeros(1,4*(total_segment-1)) final_time^3 final_time^2 final_time^1 1;
               zeros(1,4*(total_segment-1)) 3*final_time^2 2*final_time 1 0];
% concat all the matrix M together
M = [M_first_seg;M_via;M_final_seg];
%% The coefficient the matrix should equal to
y_first = [%first segment
            init_pos;
            init_vel;
            via_pos(1); 
            via_vel(1)]
y_via = [];
for i = 1:total_segment-2
    y_via = [y_via;
             via_pos(i);
             via_vel(i);
             via_pos(i+1);
             via_vel(i+1)]
end
y_final = [%final segment
            via_pos(end); 
            via_vel(end);
            final_pos;
            final_vel]
% concat all the y together
y = [y_first;y_via;y_final];
 
segmentCoeff = M^-1*y;
end


