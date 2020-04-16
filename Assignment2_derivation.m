clear
clc

syms d0 a2 a3 d5 dE real
syms Q1 Q2 Q3 Q4 Q5 real
% from Sam's function T was simplified by hand. Qs are now in radians
T01=[   cos(Q1), -sin(Q1), 0,   0;
        sin(Q1),  cos(Q1), 0,   0;
        0,    0, 1, 300;
        0,    0, 0,   1];

T12=[cos(Q2), -sin(Q2),  0, 0;
    0,            0, -1, 0;
    sin(Q2),  cos(Q2),  0, 0;
    0,             0,  0, 1];

T23=[cos(Q3 - pi/2), -sin(Q3 - pi/2), 0, 450;
    sin(Q3 - pi/2),  cos(Q3 - pi/2), 0,     0;
    0,                    0, 1,     0;
    0,                    0, 0,     1];

T34=[cos(Q4 - pi/2), -sin(Q4 - pi/2), 0, 350;
    sin(Q4 - pi/2),  cos(Q4 - pi/2), 0,   0;
     0,        0, 1,   0;
     0,        0, 0,   1];


T45 =[  cos(Q5), -sin(Q5), 0, 0;
            0,             0, 1, 0;
 -sin(Q5), -cos(Q5), 0, 0;
            0,             0, 0, 1];

T5E=[ 1, 0, 0,  0;
 0, 1, 0,  0;
 0, 0, 1, 80;
 0, 0, 0,  1];


T02=T01*T12;
T03=T02*T23;
T04=T03*T34;
T05=T04*T45;
T0E=T05*T5E;

z1 = simplify(T01 * [0;0;1;0]);
z2 = simplify(T02 * [0;0;1;0]);
z3 = simplify(T03 * [0;0;1;0]);
z4 = simplify(T04 * [0;0;1;0]);
z5 = simplify(T05 * [0;0;1;0]);
ze = simplify(T0E * [0;0;1;0]);
%%
r_3w = simplify(T03*[a3;0;0;0]);
r_2w = simplify(T02*[a2;0;0;0] + r_3w);
r_1w = simplify(T01*[0;0;0;0] + r_2w);

r_5e = simplify(T05*[0;0;0;0]);
r_we = simplify(T0E*[0;0;d5+dE;0]+r_5e);

%%
J_vw = [cross(z1(1:3),r_1w(1:3)), cross(z2(1:3),r_2w(1:3)), ...
        cross(z3(1:3),r_3w(1:3)), zeros(3,3)];
J_ww = [z1(1:3) z2(1:3) z3(1:3) z4(1:3) z5(1:3) ze(1:3)];
J_w = simplify([J_vw;J_ww]);
%%

r_we_skew = [0 r_we(3) -r_we(2) ; -r_we(3) 0 r_we(1) ; r_we(2) -r_we(1) 0 ];
J_ve = [eye(3) , r_we_skew]*J_w;
J_we = [zeros(3) , eye(3)]*J_w;

J_e = simplify([J_ve;J_we]);

%%
old=[sin(Q1),sin(Q2), sin(Q3),sin(Q4), sin(Q5), ...
    cos(Q1),cos(Q2), cos(Q3),cos(Q4),cos(Q5)];
new={"S1","S2","S3","S4","S5","C1","C2","C3","C4","C5"};
old2=[sin(Q2 + Q3),sin(Q2 + Q3 + Q4), cos(Q2 + Q3),sin(Q1 + Q2 + Q3 + Q4),cos(Q1 + Q2 + Q3 + Q4),cos(Q2 + Q3 + Q4)];
new2={"S23","S234","C23","S1234","C1234","C234"};
old3=[cos(Q2 - Q1 + Q3 + Q4), sin(Q2 - Q1 + Q3 + Q4)];
new3={"C_1234", "S_1234" };

J_w=subs(J_w, old, new);
J_w=subs(J_w, old2, new2);
J_w=subs(J_w, old3, new3)

J_e=subs(J_e, old, new);
J_e=subs(J_e, old2, new2);
J_e=subs(J_e, old3, new3)
r_3w = subs(r_3w, [old old2 old3], [new new2 new3])
r_2w = subs(r_2w, [old old2 old3], [new new2 new3])
r_1w = subs(r_1w, [old old2 old3], [new new2 new3])
r_5e = subs(r_5e, [old old2 old3], [new new2 new3])
r_we = subs(r_we, [old old2 old3], [new new2 new3])
z1 = subs(z1, [old old2 old3], [new new2 new3])
z2 = subs(z2, [old old2 old3], [new new2 new3])
z3 = subs(z3, [old old2 old3], [new new2 new3])
z4 = subs(z4, [old old2 old3], [new new2 new3])
z5 = subs(z5, [old old2 old3], [new new2 new3])
ze = subs(ze, [old old2 old3], [new new2 new3])
