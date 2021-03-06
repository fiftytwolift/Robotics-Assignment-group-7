function segmentCoeff = TrajGen01(init, final, tfinal)

% init(1,i) are the position for segment i, init(2,i) are the velocity  for segment i
% final(1,i) are the position for segment i, final(2,i) are the velocity  for segment i
% assuming the trajectory start at t = 0
num_segment = length(tfinal);
tfinal = [0 tfinal];
segmentCoeff = [];
for i = 1:num_segment
    current_segCoeff = TrajGen_each_seg(init(:,i),final(:,i),[tfinal(i) tfinal(i+1)]);
    segmentCoeff = [segmentCoeff;current_segCoeff];
end
