%% Q array and Q dot array variables
syms q1 q2 q3 q4 real
syms L0 L1 L2 L3 L4 L5 L6 real
%% DH Table Generation
% Create all the arrays in the DH table
Dx = [0;0;L3;L4;0];
Rx = [0;pi/2;-pi/2;pi;0];
Dz = [L0;-L1;0;L5+q4;L6];
Rz = [q1;q2;q3;0;0];
% Generates DH Table
DH_table = [Dx Rx Dz Rz];

%% Find the translation matrix
T0_1 = vpa(Transform_DH(0,1,DH_table),3); % To 0 from 1
T1_2 = vpa(Transform_DH(1,2,DH_table),3); % To 1 from 2
T2_3 = vpa(Transform_DH(2,3,DH_table),3); % To 2 from 3
T3_4 = vpa(Transform_DH(3,4,DH_table),3); % To 3 from 4
T4_5 = vpa(Transform_DH(4,5,DH_table),3); % To 4 from 5

%% Find the specific translation matrix
T3_0 = simplify(vpa(T0_1*T1_2*T2_3)^-1);
T3_1 = simplify(vpa(T1_2*T2_3)^-1);
T3_2 = simplify(vpa(T2_3)^-1);
T0_3 = simplify(vpa(T0_1*T1_2*T2_3));

%% Find the rotational matrix
R3_0 = T3_0(1:3,1:3)
R3_1 = T3_1(1:3,1:3)
R3_2 = T3_2(1:3,1:3)