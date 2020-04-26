function [Jacobian] = calculate_Jacobian_Differentiate_Method(q_array)

%% Robotic Systems Assignment 2
% Author: Jonathan Wong, Samuel Wong, Peter Lin
% Name: main_script.m
% Purpose: All functions and implementations will be called from here to
%          execute assignment 1 tasks.
clearvars  -except q_array
% clc

%% check whether the input is avialable
if (length(q_array) ~= 5)
    warning('q_array must be length of 5')
    return;
end

%% Parameter Definitions
% Defines all parameters of the robot
Link_0 = 0.300; %link between ground to first joint
Link_1 = 0.450; %link between joint 1 to joint 2 (Changed to 360 for Q1.6)
Link_2 = 0.350; %link between joint 2 to joint 3 (Changed to 250 for Q1.6)
Link_3 = 0.050; %link between joint 3 to joint 4 (Changed to 60 for Q1.6)
Link_4 = 0.030; %link between joint 4 to end effector

%% DH Table Definitions
% From the DH table, the variables are defined below
syms d0 a2 a3 d5 dE Q1 Q2 Q3 Q4 Q5 E t
d0 = Link_0;
a2 = Link_1;
a3 = Link_2;
d5 = Link_3;
dE = Link_4;

%% DH Table Generation
% Create all the arrays in the DH table
i_value = [1;2;3;4;5;E];
Dx = [0;0;a2;a3;0;0];
Rx = [0;pi/2;0;0;-pi/2;0];
Dz = [d0;0;0;0;0;d5+dE];
Rz = [Q1;Q2;-pi/2 + Q3;-pi/2 + Q4;Q5;0];
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
T0_E = T0_5 * T5_E;


%% Derive Jacobian
syms Q1_(t) Q2_(t) Q3_(t) Q4_(t) Q5_(t) t
syms v_val v_val_d
v_val = transpose(T0_E(13:15));
v_val = subs(v_val,Q1,Q1_(t));
v_val = subs(v_val,Q2,Q2_(t));
v_val = subs(v_val,Q3,Q3_(t));
v_val = subs(v_val,Q4,Q4_(t));
v_val = subs(v_val,Q5,Q5_(t));
v_val_d = diff(v_val,t);
v_val_d = vpa(convert_diff_t(diff(v_val,t),true),2)
syms Q1_d Q2_d Q3_d Q4_d Q5_d
syms d0 a2 a3 d5 dE Q1_ Q2_ Q3_ Q4_ Q5_ E t
vars = [Q1_d Q2_d Q3_d Q4_d Q5_d];
[Coefficient, ZerosM] = equationsToMatrix(v_val_d,vars);

%% output of a cleaned up version of jacobian
Jacobian = vpa(subs(Coefficient,[Q1_ Q2_ Q3_ Q4_ Q5_],q_array),4);
Jacobian(find(abs(Jacobian)<0.0001)) = 0;
Jacobian = vpa(Jacobian,4);
