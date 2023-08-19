function [p0, p1, p2, p3] = direct_kinematics(q)

    q1 = q(1);
    q2 = q(2);
    q3 = q(3);

    a1 = 1;
    a2 = 1;
    a3 = 1;

    p0 = [0;0;0];
    p1 = [a1*cos(q1); a1*sin(q1);0];
    p2 = p1+[a2*cos(q2); a2*sin(q2);0];
    p3 = [(a2 + a3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + a1*cos(q1);(a2 + a3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + a1*sin(q1);0];

end