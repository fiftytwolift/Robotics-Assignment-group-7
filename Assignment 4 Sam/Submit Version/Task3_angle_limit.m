function [q1,q2,q1dot,q2dot] = Task3_angle_limit(Q1_in,Q2_in,Q1dot_in,Q2dot_in)
%% passing the values straight out
q1 = real(Q1_in);
q2 = real(Q2_in);
q1dot = real(Q1dot_in);
q2dot = real(Q2dot_in);
%% Change the values that exceeded the limit
if (real(Q1_in)>pi/3)
    q1 = pi/3;
    q1dot = 0;
elseif (real(Q1_in)<-pi/3)
    q1=-pi/3;
    q1dot = 0;
end
if (real(Q2_in) > pi/2)
    q2 = pi/2;
    q2dot=0;
elseif (real(Q2_in) <-pi/2)
    q2 = -pi/2;
    q2dot = 0;
end


