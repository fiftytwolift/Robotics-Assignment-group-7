%% Robotic Systems Assignment 1
% Author: Jonathan Wong, Samuel Wong, Peter Lin
% Name: Transform_.m
% Purpose: Transform_ will take in the DH table and transform from frame1
%          to frame2

function [output_] = Transform_(frame1, frame2, DHTable)
%   Define all the matrices
syms Dx_ Rx_ Dz_ Rz_
Dx_ = [1 0 0 DHTable.Dx(frame2);
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
Rx_ = [1 0 0 0;
       0 cosd(DHTable.Rx(frame2)) -sind(DHTable.Rx(frame2)) 0;
       0 sind(DHTable.Rx(frame2)) cosd(DHTable.Rx(frame2)) 0;
       0 0 0 1];
Dz_ = [1 0 0 0;
       0 1 0 0;
       0 0 1 DHTable.Dz(frame2);
       0 0 0 1];
Rz_ = [cosd(DHTable.Rz(frame2)) -sind(DHTable.Rz(frame2)) 0 0;
       sind(DHTable.Rz(frame2)) cosd(DHTable.Rz(frame2)) 0 0;
       0 0 1 0;
       0 0 0 1];
output_ = Dx_ * Rx_ * Dz_ * Rz_;
end

