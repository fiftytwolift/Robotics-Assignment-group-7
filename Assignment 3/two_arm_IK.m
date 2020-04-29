function [Q1,Q2] = two_arm_IK(x,y,isUp)
% Q1 and Q2 are in degree
L1 = 9;
L2 = 9;
if nargin==2
    
    % from lecture 07 slide 11 - 13
    % elbow down
    Q2 = acosd((x^2+y^2 - (L1^2+L2^2))/(2*L1*L2));
    Q1 = atand(y/x) - asind((L2*sind(180-Q2))/sqrt(x^2+y^2));
    
end
if nargin==3
    if(isUp)
        % elbow up
        Q2 = -acosd((x^2+y^2 - (L1^2+L2^2))/(2*L1*L2));
        Q1 = atand(y/x) + asind((L2*sind(180+Q2))/sqrt(x^2+y^2));
    else
        % elbow down
        Q2 = acosd((x^2+y^2 - (L1^2+L2^2))/(2*L1*L2));
        Q1 = atand(y/x) - asind((L2*sind(180-Q2))/sqrt(x^2+y^2));
    end
end