function segmentCoeff = TrajGen01(init, final, tfinal)
%% checking the size and validity of the input
if (length(init) ~= 2 ||  length(final) ~= 2 || length(tfinal) ~= 2)
    warning('The input is not valid')
    return;
end

%% Setup the variables
init_pos = init(1);
init_vel = init(2);
final_pos = final(1);
final_vel = final(2);
init_time = 0;
via_time = tfinal(1);
final_time = tfinal(2);
via_x_pos = 5;
via_y_pos = 8;
via_vel = 0.25; %given

%% set up a matrix to represent the cubic spline
%the equation are written in the form a*t^3 + b*t^2 + c*t + d = c 
M = [init_time^3 init_time^2 init_time^1 1 zeros(1,4);
     3*init_time^2 2*init_time 1 0 zeros(1,4);
     via_time^3 via_time^2 via_time^1 1 zeros(1,4);
     3*via_time^2 2*via_time 1 0 zeros(1,4);
     
     zeros(1,4) via_time^3 via_time^2 via_time^1 1;
     zeros(1,4) 3*via_time^2 2*via_time 1 0;
     zeros(1,4) final_time^3 final_time^2 final_time^1 1;
     zeros(1,4) 3*final_time^2 2*final_time 1 0;]

 %% The coefficient the matrix should equal to
 y = [%first segment
     init_pos;
     init_vel;
     via_x_pos; %when calculating for x
     %via_y_pos; %when calculating for y
     via_vel;
     
     %second segment
     via_x_pos; %when calculating for x
     %via_y_pos; %when calculating for y
     via_vel;
     final_pos;
     final_vel;]
segmentCoeff = M^-1*y; 
end



