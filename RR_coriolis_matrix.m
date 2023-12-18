function C = RR_coriolis_matrix(q, dq)

% Basic params
g = 9.81;

% Manipulator params
m1 = 1;
m2 = 1;

a1=1;
a2=1;

l1=a1/2;
l2=a2/2;

Izz1 = (1/12)*m1*a1^2;
Izz2 = (1/12)*m2*a2^2;

% Basic expressions for dynamics
Alpha = Izz1 + Izz2 + m1*(l1)^2 + m2*(a1^2+l2^2);
Beta = m2*a1*l2;
Delta = Izz2 +m2*l2^2;

% Coriolis matrix
C = [-Beta*sin(q(2))*dq(2), -Beta*sin(q(2))*(dq(1)+dq(2));
     Beta*sin(q(2))*dq(1),   0];

end

