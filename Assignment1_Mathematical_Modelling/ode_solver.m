function dqdt = ode_solver(t, y, tau)
    m1 = 1; % mass
    m2 = 1;
    g = -10; % gravity
    a1 = 1; % link length
    a2 = 1;
    
    q = y(1:2);
    qdot = y(3:4);

    V = [(-m2 * a1 * a2 * (2 * qdot(1) * qdot(2) + qdot(2) * qdot(2)) * sin(q(2)));
        m2 * a1 * a2 * q(1) * q(1) * sin(q(2))];

    G = [(m1 + m2) * g * a1 * cos(q(1)) + m2 * g * a2 * cos(q(1) + q(2));
        m2 * g * a2 * cos(q(1) + q(2))];

    M = [(m1 + m2) * a1 * a1 + m2 * a2 * a2 + 2 * m2 * a1 * a2 * cos(q(2)), m2 * a2 * a2 + m2 * a1 * a2 * cos(q(2));
        m2 * a2 * a2 + m2 * a1 * a2 * cos(q(2)),  m2 * a2 * a2];
    
    qddot = M \ (-V - G + tau);

    dqdt = [qdot; qddot];
end


