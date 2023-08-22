function G = RRR_gravity_matrix(q)
q1 = q(1);
q2 = q(2);
q3 = q(3);
G = [- (981*sin(q1 + q2))/50 - (2943*sin(q1))/100 - (981*sin(q1 + q2)*cos(q3))/100; - (981*sin(q1 + q2))/50 - (981*sin(q1 + q2)*cos(q3))/100; -(981*cos(q1 + q2)*sin(q3))/100];