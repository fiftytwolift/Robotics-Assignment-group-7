%% Robotic Systems Assignment 1
% Author: Jonathan Wong, Samuel Wong, Peter Lin
% Name: Transform_.m
% Purpose: Transform_ will take in the DH table and transform from frame1
%          to frame2

function [output] = Transform_DH(frame1, frame2, DHTable)
%   Define all the matrices

Dx = [1 0 0 DHTable(frame2,1);
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
Rx = [1 0 0 0;
       0 cos(DHTable(frame2,2)) -sin(DHTable(frame2,2)) 0;
       0 sin(DHTable(frame2,2)) cos(DHTable(frame2,2)) 0;
       0 0 0 1];
Dz = [1 0 0 0;
       0 1 0 0;
       0 0 1 DHTable(frame2,3);
       0 0 0 1];
Rz = [cos(DHTable(frame2,4)) -sin(DHTable(frame2,4)) 0 0;
       sin(DHTable(frame2,4)) cos(DHTable(frame2,4)) 0 0;
       0 0 1 0;
       0 0 0 1];
output = Dx * Rx * Dz * Rz;
end

