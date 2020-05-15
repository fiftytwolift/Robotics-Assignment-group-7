%% Parameter Definitions
% Defines all parameters of the robot
syms m1 m2 Izz1 Izz2 L1 L2 rC1 rC2 g
    m1 = 2;
    m2 = 1;
    Izz1 = 0.5;
    Izz2 = 0.3;
    L1 = 1;
    L2 = 0.6;
    rC1 = L1/2;
    rC2 = L2/2;
    g = 9.81;

IC1 = [0 0 0;
    0 0 0;
    0 0 Izz1];
IC2 = [0 0 0;
    0 0 0;
    0 0 Izz2];
%% Q array and Q dot array variables
syms Q1 Q2 Q1_dot Q2_dot

%% DH Table Generation
% Create all the arrays in the DH table
Dx = [0;L1;L2];
Rx = [0;0;0];
Dz = [0;0;0];
Rz = [Q1;Q2;0];
% Generates DH Table
DH_table = table(Dx,Rx,Dz,Rz);

%% Find frame {E}
syms T0_1 T1_2 T2_E
T0_1 = Transform_(0,1,DH_table);
T1_2 = Transform_(1,2,DH_table);
T2_E = Transform_(2,3,DH_table);

T0_2 = T0_1* T1_2;
T0_E = simplify(T0_2 * T2_E)

%% getting the vector pointing from 1,2 to centre of mass
p_2_e = T0_2*[L2;0;0;0];

p_1_e = T0_1*[L1;0;0;0] + p_2_e;
p_1_1 = T0_1*[0;0;0;0];

%% Getting the z axis expressed in frame 0
z1 = T0_1 * [0;0;1;0];
z2 = T0_2 * [0;0;1;0];
ze = T0_E * [0;0;1;0];
%% Get the Jacobian for centre of masses
J_ve = [cross(transpose(z1(1:3)),transpose(p_1_e(1:3)))', ...
    cross(transpose(z2(1:3)),transpose(p_2_e(1:3)))'];
w_ve = [z1(1:3) z2(1:3)];
inversed = simplify(transpose([J_ve;w_ve]));
vpa(subs(J_ve(1:2,1:2),[Q1 Q2],[0 0]))*[0;2.2]