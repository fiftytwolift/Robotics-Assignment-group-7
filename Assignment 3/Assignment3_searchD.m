function Assignment3_searchD(considerArm)
%% set up variables
close all
clearvars -except considerArm

% definition of the obstacle
circle_xy = [10 7];
r = 1.5;
% define the obstacle circle
index = 0;
for i = 0:0.01:3*pi
    index = index+1;
    circle_x(index) = 10 + r*cos(i);
    circle_y(index) = 7 +  r*sin(i);
end
% initiate variables for the search counter
min_path_dist = inf;
min_d_xpos = inf;
min_d_ypos = inf;
discard = false;

%% perform a grid search around the area of interest
for dx = -5:0.1:5
    for dy = -5:0.1:5
        d_xpos = circle_xy(1) + dx;
        d_ypos = circle_xy(2) + dy;
        path_dist = pdist([d_xpos,d_ypos;14,7],'euclidean') + pdist([d_xpos,d_ypos;5,8],'euclidean');
        %ignore this via point because it is longer than current best
        if path_dist > min_path_dist
            continue;
        end
        
        % timing, position and velocity for pt B, pt C and pt D
        time = [0,3,5.5,8];
        ptA = [2,5;0,0];
        ptB = [5,8;0.25,0.25];
        ptC = [14,7;0,0];
        ptD = [d_xpos,d_ypos;0,0];
        %% calculate the segments coefficents for x and y
        x_coeff_1 = TrajGen01(ptA(:,1),ptB(:,1),time(1:2));
        x_coeff_2 = TrajGen01(ptB(:,1),ptD(:,1),time(2:3));
        x_coeff_3 = TrajGen01(ptD(:,1),ptC(:,1),time(3:4));
        y_coeff_1 = TrajGen01(ptA(:,2),ptB(:,2),time(1:2));
        y_coeff_2 = TrajGen01(ptB(:,2),ptD(:,2),time(2:3));
        y_coeff_3 = TrajGen01(ptD(:,2),ptC(:,2),time(3:4));
        x_coeff = [x_coeff_1;x_coeff_2;x_coeff_3];
        y_coeff = [y_coeff_1;y_coeff_2;y_coeff_3];
        
        %% get the displacement and velocity in both x and y
        x_displacement = plot_displacement(x_coeff,time(2:4),false);
        y_displacement = plot_displacement(y_coeff,time(2:4),false);
        
        %% check whether we collided with the obstacle
        closest_dist_to_obs = inf;
        
        for i = 1:length(x_displacement)
            dist = sqrt( (x_displacement(i) - circle_xy(1))^2 + (y_displacement(i) - circle_xy(2))^2);
            if dist < closest_dist_to_obs
                closest_dist_to_obs = dist;
            end
        end
        %discard this result if there is a collison
        if closest_dist_to_obs < r
            discard = true;
        end
        
        % consider the robotic arm
        if (considerArm)
            L1 = 9;
            L2 = 9;
            division = 20;
            Q1 = [];
            Q2 = [];
            for i = 1:length(x_displacement)
                [Q1,Q2] = two_arm_IK(x_displacement(i),y_displacement(i));
                mid_pt = [L1*cosd(Q1),L1*sind(Q1)];
                end_effector = mid_pt+[L2*cosd(Q1+Q2),L2*sind(Q1+Q2)];
                arm_pos = [linspace(0,mid_pt(1),division) linspace(mid_pt(1),end_effector(1),division);
                    linspace(0,mid_pt(2),division) linspace(mid_pt(2),end_effector(2),division)];
                for j = 1:length(arm_pos)
                    % if the robotic arm collides with the obstacle
                    if pdist([arm_pos(1,j),arm_pos(2,j);circle_xy(1),circle_xy(2)],'euclidean') < r
                        discard = true;
                        break;
                    end
                end
                if (discard)
                    break;
                end
            end
        end
        
        
        if discard == false
            min_path_dist = path_dist;
            min_d_xpos = d_xpos;
            min_d_ypos = d_ypos;
            fprintf('current best via point is (%4.2f , %4.2f) with BCD linaer distance %4.2f \n',min_d_xpos, min_d_ypos, min_path_dist)
        end
        % reset the discard
        discard =  false;
    end
end