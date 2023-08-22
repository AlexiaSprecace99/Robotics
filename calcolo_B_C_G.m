%In this Script there is the computation of B,C and G matrix
syms q1 q2 q3 dq1 dq2 dq3 
assume(q1,'real');
assume(q2,'real');
assume(q3,'real');
assume(dq1,'real');
assume(dq2,'real');
assume(dq3,'real');

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

Jg1 = [Ixx1 0 0;0 Iyy1 0;0 0 Izz1];
Jg2 = [Ixx2 0 0;0 Iyy2 0;0 0 Izz2];
Jg3 = [Ixx3 0 0;0 Iyy3 0;0 0 Izz3];

Jp1 = [-a1*sin(q1),0,0;a1*cos(q1),0,0;0,0,0];

Jp2 = [- a1*sin(q1) - a2*sin(q1+q2), - a2*sin(q1+q2),0; ...
        a1*cos(q1) + a2*cos(q1+q2),   a2*cos(q1+q2),0; 0,0,0];


Jp3 = [- a1*sin(q1) - a3*cos(q3)*sin(q1+q2) - a2*sin(q1+q2), -a3*cos(q3)*sin(q1+q2) - a3*sin(q1+q2), -a3*sin(q3)*cos(q1+q2); ...
    a1*cos(q1) + a3*cos(q3)*cos(q1+q2) + a2*cos(q1+q2),   a3*cos(q3)*cos(q1+q2) + a2*cos(q1+q2),  -a3*sin(q3)*sin(q1+q2); ...
    0,0,- a3*cos(q3)*(sin(q1+q2))^2 - a3*cos(q3)*(cos(q1+q2))^2];

J01 = [0,0,0;0,0,0;1,0,0];
J02 = [0,0,0;0,0,0;1,1,0];
J03 = [0,0,-sin(q1+q2); 0,0,cos(q1+q2);1,1,0];

R01 = [cos(q1) -sin(q1) 0;sin(q1) cos(q1) 0;0 0 1];
R02 = [cos(q1+q2), 0,-sin(q1+q2);sin(q1+q2),  0,cos(q1+q2);0,-1,0];
R03 = [cos(q3)*cos(q1+q2), -sin(q3)*cos(q1+q2),-sin(q1+q2); ...
        cos(q3)*cos(q1+q2), -sin(q3)*sin(q1+q2), cos(q1+q2); -sin(q3), -cos(q3),  0];

Jg1_0 = R01*Jg1*R01';
Jg2_0 = R02*Jg2*R02';
Jg3_0 = R03*Jg3*R03';

Bp1 = m1*Jp1'*Jp1;
Bp2 = m2*Jp2'*Jp2;
Bp3 = m3*Jp3'*Jp3;
B01 = J01'*Jg1_0*J01;
B02 = J02'*Jg2_0*J02;
B03 = J03'*Jg3_0*J03;
B = Bp1+Bp2+Bp3+B01+B02+B03;

C = sym(zeros(3,3));
q = [q1; q2;q3];
dq = [dq1;dq2;dq3];

% Calcolo dei simboli di Cristoffel

for i = 1:3
    for j = 1:3
        for k = 1:3
            C_matrix(i, j , k) = 0.5 * ((jacobian(B(i,j),q(k))) + (jacobian(B(i,k),q(j))) - (jacobian(B(j,k),q(i)))) * dq(k);

        end
    end
end

 for i = 1:3
    for j = 1:3
        C(i, j) = C_matrix(i, j, 1) + C_matrix(i, j, 2) + C_matrix(i, j, 3);

    end
 end

g = [-9.81 0 0]';
G = -(m1*Jp1'*g+m2*Jp2'*g+m3*Jp3'*g);
