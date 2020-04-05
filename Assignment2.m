%% Robotic Systems Assignment 1
% Author: Jonathan Wong, Samuel Wong, Peter Lin
% Name: main_script.m
% Purpose: All functions and implementations will be called from here to
%          execute assignment 1 tasks.
clear all
clc
%% Parameter Definitions
% Defines all parameters of the robot
Link_0 = 300; %link between ground to first joint
Link_1 = 450; %link between joint 1 to joint 2 (Changed to 360 for Q1.6)
Link_2 = 350; %link between joint 2 to joint 3 (Changed to 250 for Q1.6)
Link_3 = 50; %link between joint 3 to joint 4 (Changed to 60 for Q1.6)
Link_4 = 30; %link between joint 4 to end effector

%% DH Table Definitions
% From the DH table, the variables are defined below
syms d0 a2 a3 d5 dE Q1 Q2 Q3 Q4 Q5 E 
d0 = Link_0;
a2 = Link_1;
a3 = Link_2;
d5 = Link_3;
dE = Link_4;
% % For Zero Configuration, all zeros
% Q1 = 0;
% Q2 = 0; %Change this to 90 for Straight up
% Q3 = 0; %Change this to 90 for L-shape and Straight up configurations
% Q4 = 0;
% Q5 = 0;

%% DH Table Generation
% Create all the arrays in the DH table
i_value = [1;2;3;4;5;E];
Dx = [0;0;a2;a3;0;0];
Rx = [0;90;0;0;-90;0];
Dz = [d0;0;0;0;0;d5+dE];
Rz = [Q1;Q2;-90 + Q3;-90 + Q4;Q5;0];
% Generates DH Table
DH_table = table(i_value,Dx,Rx,Dz,Rz);
DH_table(1:6,:);

%% Find frame {E}
syms T0_1 T1_2 T2_3 T3_4 T4_5 T5_E
T0_1 = Transform_(0,1,DH_table);
T1_2 = Transform_(1,2,DH_table);
T2_3 = Transform_(2,3,DH_table);
T3_4 = Transform_(3,4,DH_table);
T4_5 = Transform_(4,5,DH_table);
T5_E = Transform_(5,6,DH_table);

T0_2 = T0_1* T1_2;
T0_3 = T0_2 * T2_3;
T0_4 = T0_3 * T3_4;
T0_5 = T0_4 * T4_5;
T0_E = T0_5*T5_E;

%% getting the vector pointing from 1,2,3 to point W
p_3w = T0_3*[a3;0;0;0];
p_2w = T0_2*[a2;0;0;0] + p_3w;
p_1w = T0_1*[0;0;0;0] + p_2w;

p_5e = T0_5*[0;0;0;0];
p_we = T0_E*[0;0;d5+dE;0]+p_5e;
%% Getting the z vector expressed in frame 0
z1 = T0_1 * [0;0;1;0];
z2 = T0_2 * [0;0;1;0];
z3 = T0_3 * [0;0;1;0];
z4 = T0_4 * [0;0;1;0];
z5 = T0_5 * [0;0;1;0];
ze = T0_E * [0;0;1;0];
%% Get the Jacobians for w
J_vw = [cross(z1(1:3)',p_1w(1:3)')', cross(z2(1:3)',p_2w(1:3)')', cross(z3(1:3)',p_3w(1:3)')' zeros(3,3)];
J_ww = [z1(1:3) z2(1:3) z3(1:3) z4(1:3) z5(1:3) ze(1:3)];


%% Get the Jacobian for frame e
syms Q1_dot Q2_dot Q3_dot Q4_dot Q5_dot
J_ve = J_vw +cross(p_we(1:3)',(J_ww*[Q1_dot;Q2_dot;Q3_dot;Q4_dot;Q5_dot;0]))';
J_we = J_ww;