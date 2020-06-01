function dydt = Task2_dynamics(t,y,tau)

    % x represents the state that contains q and qdot for the robot 
    % ie. for both joints
    syms Q1 Q2
    q = [y(1); y(2)];
    qdot = [y(3); y(4)];
    q1 = y(1);
    q2 = y(2);
    q1dot = y(3);
    q2dot = y(4);
    
    %% Variable Declaration
    L_1 = 1; % length of link 1 /m
    L_2 = 0.6; % length of link 2 /m
    r_c1 = L_1/2; % distance from origin of link 1 to CoM for link 1 /m
    r_c2 = L_2/2; % distance from origin of link 2 to CoM for link 2 /m
    m_1 = 2; % mass of link 1
    m_2 = 1; % mass of link 2
    I_1 = [0 0 0;
           0 0 0;
           0 0 0.5];  % angular moment of inertia for link 1 about CoM
    I_2 = [0 0 0;
           0 0 0;
           0 0 0.3];  % angular moment of inertia for link 2 about CoM
    g = 9.81; % Acceleration due to gravity
    
    % friction
    friction_coeff = 0.1;
    friction = friction_coeff * [q1dot ; q2dot];
    
	
    %% Equation of Motion Terms
    % inertia matrix
    A = [m_2*L_1^2 + 2*m_2*cos(q2)*L_1*r_c2 + m_1*r_c1^2 + m_2*r_c2^2 + I_1(3,3)+I_2(3,3), m_2*r_c2^2 + L_1*m_2*cos(q2)*r_c2 + I_2(3,3);
         m_2*r_c2^2 + L_1*m_2*cos(q2)*r_c2 + I_2(3,3),m_2*r_c2^2 + I_2(3,3)];
    % Coriolis and  centrifugal
    B = [ -2*L_1*m_2*r_c2*sin(q2);
          L_1*m_2*r_c2*sin(q2)];
	C = [ 0, -L_1*m_2*r_c2*sin(q2);
          L_1*m_2*r_c2*sin(q2),0];
    % gravity term
    G = [g*m_2*(r_c2*cos(q1 + q2) + L_1*cos(q1)) + g*m_1*r_c1*cos(q1);
         g*m_2*r_c2*cos(q1 + q2)];

    %% Acceleration Calculatiuon
    % Calculating the qdoubledot -- joint space acceleration
    qddot = inv(A) * (tau - friction - B * (q1dot *q2dot) - C * [q1dot^2; q2dot^2] - G); % this is qdoubledot.
    
    %% Output
    dydt = zeros(4,1);
    dydt(1) = y(3);
    dydt(2) = y(4);
    dydt(3) = qddot(1);
    dydt(4) = qddot(2);
    
end