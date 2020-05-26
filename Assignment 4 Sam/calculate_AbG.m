function [A,b,G] = calculate_AbG(q_array,q_dot_array, is_Symbolic)
%% Parameter Definitions
% Defines all parameters of the robot
syms m1 m2 Izz1 Izz2 L1 L2 rC1 rC2 g real
if is_Symbolic == false
    m1 = 2;
    m2 = 1;
    Izz1 = 0.5;
    Izz2 = 0.3;
    L1 = 1;
    L2 = 0.6;
    rC1 = L1/2;
    rC2 = L2/2;
    g = 9.81;
end

IC1 = [0 0 0;
    0 0 0;
    0 0 Izz1];
IC2 = [0 0 0;
    0 0 0;
    0 0 Izz2];
%% Q array and Q dot array variables
Q1 = q_array(1);
Q2 = q_array(2);
Q1_dot = q_dot_array(1);
Q2_dot = q_dot_array(2);
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
T0_1 = Transform_(0,1,DH_table)
T1_2 = Transform_(1,2,DH_table)
T2_E = Transform_(2,3,DH_table)

T0_2 = T0_1* T1_2
T0_E = T0_2 * T2_E

%% getting the vector pointing from 1,2 to centre of mass
p_2_2mid = T0_2*[rC2;0;0;0]

p_1_2mid = T0_1*[L1;0;0;0] + p_2_2mid
p_1_1mid = T0_1*[rC1;0;0;0]

%% Getting the z axis expressed in frame 0
z1 = T0_1 * [0;0;1;0]
z2 = T0_2 * [0;0;1;0]

%% Get the Jacobian for centre of masses
J_v1 = [cross(transpose(z1(1:3)),transpose(p_1_1mid(1:3)))', zeros(3,1)]
J_w1 = [z1(1:3), zeros(3,1)]

J_v2 = simplify([cross(transpose(z1(1:3)),transpose(p_1_2mid(1:3)))', ...
    cross(transpose(z2(1:3)),transpose(p_2_2mid(1:3)))'])
J_w2 = [z1(1:3) z2(1:3)]

%% Calculate matrix A
A = m1*transpose(J_v1)*J_v1 + transpose(J_w1)*IC1*J_w1 + ...
    m2*transpose(J_v2)*J_v2 + transpose(J_w2)*IC2*J_w2;

%% Calculate matrix b
syms a_nn
a_nn(1,1,1) = 0;
a_nn(1,2,1) = 0;
a_nn(2,1,1) = 0;
a_nn(2,2,1) = 0;

a_nn(1,1,2) = -2*L1*m2*rC2*sin(Q2);
a_nn(1,2,2) = -L1*m2*rC2*sin(Q2);
a_nn(2,1,2) = -L1*m2*rC2*sin(Q2);
a_nn(2,2,2) = 0;

for i=1:2
    for j = 1:2
        for k = 1:2
            b(i,j,k) = 0.5*(a_nn(i,j,k) + a_nn(i,k,j) - a_nn(j,k,i));
        end
    end
end
B = [2*b(1,1,2); 2*b(2,1,2)];
C = [b(1,1,1) b(1,2,2);b(2,1,1) b(2,2,2)];
b = B*[Q1_dot*Q2_dot] + C*[Q1_dot^2;Q2_dot^2];

%% Calculate matrix G
G = [transpose(J_v1)*m1*g*[0;1;0] + transpose(J_v2)*m2*g*[0;1;0]];
%% All variables output
if is_Symbolic==true
    A = simplify(A)
    B = simplify(B)    
    C = simplify(C)    
    b = simplify(b)
    G = simplify(G)
else
    A;
    B;
    C;
    b;
    G;
end

% A
% b
% G
