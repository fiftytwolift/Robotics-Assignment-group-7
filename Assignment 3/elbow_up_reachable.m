figure()
discard = false;
%% variables for the robot
L1 = 9;
L2 = 9;
division = 10;
%% draw the obstacle circle
r = 1.5;
circle_xy = [10 7];
index = 0;
for i = 0:0.01:3*pi
    index = index+1;
    circle_x(index) = 10 + r*cos(i);
    circle_y(index) = 7 +  r*sin(i);
end
for Q1=30:10:90
    for Q2 = -90:10:0
        hold on
        mid_pt = [L1*cosd(Q1),L1*sind(Q1)];
        end_effector = mid_pt+[L2*cosd(Q1+Q2),L2*sind(Q1+Q2)];
        arm_pos = [linspace(0,mid_pt(1),division) linspace(mid_pt(1),end_effector(1),division);
            linspace(0,mid_pt(2),division) linspace(mid_pt(2),end_effector(2),division)];
        for j = 1:length(arm_pos)
            % if the robotic arm collides with the obstacle
            if pdist([arm_pos(1,j),arm_pos(2,j);circle_xy(1),circle_xy(2)],'euclidean') < r
                discard = true;
            end
        end
        if (~discard)
            plot_robot(Q1,Q2)
        end
        discard = false;
    end
end



hold on
plot(circle_x,circle_y,'-')