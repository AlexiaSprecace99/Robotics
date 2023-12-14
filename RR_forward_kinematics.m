function [p0,p1,p2] = RR_forward_kinematics(q)

    q1 = q(1);
    q2 = q(2);

    a1 = 1;
    a2 = 1;

    p0 = [0;0];
    p1 = [a1*cos(q1); a1*sin(q1)];
    p2 = p1 + [a2*cos(q1+q2); a2*sin(q1+q2)];
end