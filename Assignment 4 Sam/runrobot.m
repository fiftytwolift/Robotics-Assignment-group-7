function dydt = runrobot(t,y, tau)
    % x represents the state that contains q and qdot for the robot 
    % ie. for both joints
    q = [y(1); y(2)];
    qdot = [y(3); y(4)];
    q1 = y(1);
    q2 = y(2); 
    q1dot = y(3);
    d2dot = y(4);

    m1 = 2.00;
    m2 = 1;
    Izz1 = 0.5;
    Izz2 = 0.3;
    L1 = 1;
    L2 = 0.6;
    rC1 = L1/2;
    rC2 = L2/2;
    g = 9.81;

    % inertia matrix
    A = [ m2*L1^2 + 2*m2*cos(q2)*L1*rC2 + m1*rC1^2 + m2*rC2^2 + Izz1 + Izz2, m2*rC2^2 + L1*m2*cos(q2)*rC2 + Izz2;
          m2*rC2^2 + L1*m2*cos(q2)*rC2 + Izz2, m2*rC2^2 + Izz2];
    % Coriolis and  centrifugal
    B = [-2*L1*m2*rC2*sin(q2); 0];
    C = [0, -L1*m2*rC2*sin(q2); L1*m2*rC2*sin(q2), 0];

    % gravity term
    G = [g*m2*(rC2*cos(q1 + q2) + L1*cos(q1)) + g*m1*rC1*cos(q1); g*m2*rC2*cos(q1 + q2)];

    % friction
    friction_coeff = 1;
    friction = friction_coeff * [q1dot ; d2dot];
    
    %Calculating the qdoubledot -- joint space acceleration
    qddot = inv(A) * (tau - friction - B * (q1dot*d2dot) - C * [q1dot^2; d2dot^2] - G); % this is qdoubledot.

    dydt = zeros(4,1);
    dydt(1) = y(3);
    dydt(2) = y(4);
    dydt(3) = qddot(1);
    dydt(4) = qddot(2);
    
end