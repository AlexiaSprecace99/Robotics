function B = RRR_mass_matrix(q)
q1 = q(1);
q2 = q(2);
q3 = q(3);

%parameters
a1 = 1;
a2 = 1;
a3 = 1;

m1 = 1;
m2 = 1;
m3 = 1;

l1 = a1/2;
l2 = a2/2;
l3 = a3/2;

Ixx1 = 1/12*m1*a1^2;
Iyy1 = Ixx1;
Izz1 = Ixx1;

Ixx2 = 1/12*m2*a2^2;
Iyy2 = Ixx2;
Izz2 = Ixx2;

Ixx3 = 1/12*m3*a3^2;
Iyy3 = Ixx3;
Izz3 = Ixx3;

b11 = Ixx3 + Iyy2 + Izz1 + a1^2*m2 + a1^2*m3 + a2^2*m3 + l1^2*m1 + l2^2*m2 - Ixx3*cos(q3)^2 + Iyy3*cos(q3)^2 + l3^2*m3*cos(q3)^2 + 2*a1*a2*m3*cos(q2) + 2*a1*l2*m2*cos(q2) + 2*a2*l3*m3*cos(q3) + 2*a1*l3*m3*cos(q2)*cos(q3);
b12 = Ixx3 + Iyy2 + a2^2*m3 + l2^2*m2 - Ixx3*cos(q3)^2 + Iyy3*cos(q3)^2 + l3^2*m3*cos(q3)^2 + a1*a2*m3*cos(q2) + a1*l2*m2*cos(q2) + 2*a2*l3*m3*cos(q3) + a1*l3*m3*cos(q2)*cos(q3);
b13 = (Ixx3*cos(2*q1)*sin(2*q2)*sin(2*q3))/4 - (Ixx3*cos(2*q1)*cos(2*q2)*sin(2*q3))/4 - (Ixx3*sin(2*q3))/4 + (Ixx3*cos(2*q2)*sin(2*q1)*sin(2*q3))/4 + (Ixx3*sin(2*q1)*sin(2*q2)*sin(2*q3))/4 - a1*l3*m3*sin(q2)*sin(q3);
b21 = Ixx3 + Iyy2 + a2^2*m3 + l2^2*m2 - Ixx3*cos(q3)^2 + Iyy3*cos(q3)^2 + l3^2*m3*cos(q3)^2 + a1*a2*m3*cos(q2) + a1*l2*m2*cos(q2) + 2*a2*l3*m3*cos(q3) + a1*l3*m3*cos(q2)*cos(q3);
b22 = Ixx3 + Iyy2 + a2^2*m3 + l2^2*m2 - Ixx3*cos(q3)^2 + Iyy3*cos(q3)^2 + l3^2*m3*cos(q3)^2 + 2*a2*l3*m3*cos(q3);
b23 = -Ixx3*cos(q1 + q2)*cos(q3)*sin(q3)*(cos(q1 + q2) - sin(q1 + q2));
b31 = (Ixx3*cos(2*q1)*sin(2*q2)*sin(2*q3))/4 - (Ixx3*cos(2*q1)*cos(2*q2)*sin(2*q3))/4 - (Ixx3*sin(2*q3))/4 + (Ixx3*cos(2*q2)*sin(2*q1)*sin(2*q3))/4 + (Ixx3*sin(2*q1)*sin(2*q2)*sin(2*q3))/4 - a1*l3*m3*sin(q2)*sin(q3);
b32 = -Ixx3*cos(q1 + q2)*cos(q3)*sin(q3)*(cos(q1 + q2) - sin(q1 + q2));
b33 = Izz3 + l3^2*m3 + Ixx3*cos(q1 + q2)^2*cos(q3)^2 - 2*Ixx3*cos(q1 + q2)^3*sin(q1 + q2)*cos(q3)^2;
 
B = [b11 b12 b13 ; b21 b22 b23 ; b31 b32 b33];

% b11 = (cos(q1 + q2) + cos(q1) + cos(q1 + q2)*cos(q3))^2 + cos(q1)^2 + cos(q3)^2/12 + (cos(q1 + q2) + cos(q1))^2 + (sin(q1 + q2) + sin(q1) + sin(q1 + q2)*cos(q3))^2 + sin(q1)^2 + sin(q3)^2/12 + (sin(q1 + q2) + sin(q1))^2 + 1/6;
% b12 = sin(q1 + q2)*(sin(q1 + q2) + sin(q1)) + cos(q3)^2/12 + (sin(q1 + q2) + sin(q1 + q2)*cos(q3))*(sin(q1 + q2) + sin(q1) + sin(q1 + q2)*cos(q3)) + sin(q3)^2/12 + cos(q1 + q2)*(cos(q1 + q2) + cos(q1)) + (cos(q1 + q2) + cos(q1 + q2)*cos(q3))*(cos(q1 + q2) + cos(q1) + cos(q1 + q2)*cos(q3)) + 1/12;
% b13 = cos(q1 + q2)*sin(q3)*(sin(q1 + q2) + sin(q1) + sin(q1 + q2)*cos(q3)) - sin(q1 + q2)*sin(q3)*(cos(q1 + q2) + cos(q1) + cos(q1 + q2)*cos(q3)) - cos(q1 + q2)*((cos(q1 + q2)*cos(q3)*sin(q3))/12 - (sin(q1 + q2)*cos(q3)*sin(q3))/12);
% b21 = sin(q1 + q2)*(sin(q1 + q2) + sin(q1)) + cos(q3)^2/12 + (sin(q1 + q2) + sin(q1 + q2)*cos(q3))*(sin(q1 + q2) + sin(q1) + sin(q1 + q2)*cos(q3)) + sin(q3)^2/12 + cos(q1 + q2)*(cos(q1 + q2) + cos(q1)) + (cos(q1 + q2) + cos(q1 + q2)*cos(q3))*(cos(q1 + q2) + cos(q1) + cos(q1 + q2)*cos(q3)) + 1/12;
% b22 = cos(q1 + q2)^2 + sin(q1 + q2)^2 + cos(q3)^2/12 + (cos(q1 + q2) + cos(q1 + q2)*cos(q3))^2 + sin(q3)^2/12 + (sin(q1 + q2) + sin(q1 + q2)*cos(q3))^2 + 1/12;
% b23 = cos(q1 + q2)*sin(q3)*(sin(q1 + q2) + sin(q1 + q2)*cos(q3)) - sin(q1 + q2)*sin(q3)*(cos(q1 + q2) + cos(q1 + q2)*cos(q3)) - cos(q1 + q2)*((cos(q1 + q2)*cos(q3)*sin(q3))/12 - (sin(q1 + q2)*cos(q3)*sin(q3))/12);
% b31 = cos(q1 + q2)*sin(q3)*(sin(q1 + q2) + sin(q1) + sin(q1 + q2)*cos(q3)) - sin(q1 + q2)*sin(q3)*(cos(q1 + q2) + cos(q1) + cos(q1 + q2)*cos(q3)) - cos(q1 + q2)*((cos(q1 + q2)*cos(q3)*sin(q3))/12 - (sin(q1 + q2)*cos(q3)*sin(q3))/12);
% b32 = cos(q1 + q2)*sin(q3)*(sin(q1 + q2) + sin(q1 + q2)*cos(q3)) - sin(q1 + q2)*sin(q3)*(cos(q1 + q2) + cos(q1 + q2)*cos(q3)) - cos(q1 + q2)*((cos(q1 + q2)*cos(q3)*sin(q3))/12 - (sin(q1 + q2)*cos(q3)*sin(q3))/12);
% b33 = cos(q1 + q2)*(cos(q1 + q2)*(cos(q1 + q2)^2/12 + (cos(q1 + q2)^2*cos(q3)^2)/12 + (sin(q1 + q2)^2*sin(q3)^2)/12) - sin(q1 + q2)*((cos(q1 + q2)^2*cos(q3)^2)/12 - (cos(q1 + q2)*sin(q1 + q2))/12 + (cos(q1 + q2)*sin(q1 + q2)*sin(q3)^2)/12)) + (cos(q3)*cos(q1 + q2)^2 + cos(q3)*sin(q1 + q2)^2)^2 + sin(q1 + q2)*(sin(q1 + q2)*(sin(q1 + q2)^2/12 + (cos(q1 + q2)^2*cos(q3)^2)/12 + (cos(q1 + q2)^2*sin(q3)^2)/12) - cos(q1 + q2)*((cos(q1 + q2)^2*cos(q3)^2)/12 - (cos(q1 + q2)*sin(q1 + q2))/12 + (cos(q1 + q2)*sin(q1 + q2)*sin(q3)^2)/12)) + cos(q1 + q2)^2*sin(q3)^2 + sin(q1 + q2)^2*sin(q3)^2;
% 
% B=[b11,b12,b13;b21,b22,b23;b31,b32,b33];
