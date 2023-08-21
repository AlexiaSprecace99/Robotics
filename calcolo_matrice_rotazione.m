syms q1 a1 q2 q3 a2 a3 dq1 dq2 dq3;
% q = [q1; q2;q3]; dq = [dq1; dq2;dq3];
% [~, ~, p] = direct_kinematics([q1, q2, q3]);
% J = jacobian(p,q);
T01 = [cos(q1) -sin(q1) 0 a1*cos(q1);sin(q1) cos(q1) 0 a1*sin(q1);0 0 1 0;0 0 0 1];

T12= [-sin(q2) 0 cos(q2) 0;cos(q2) 0 sin(q2) 0;0 1 0 0;0 0 0 1];

T23 = [cos(q3) -sin(q3) 0 0;sin(q3) cos(q3) 0 0;0 0 1 a2+a3;0 0 0 1];

T03 = T01*T12*T23;
d = T03(1:3,4);
analitico = jacobian(d,[q1;q2;q3]);

T02 = T01*T12;
T13 = T12*T23;

%calcolo il prodotto vettoriale tra k2 e 02-03
k2 = T02(1:3,3);
O2O3 = T03(1:3,4) - T02(1:3,4);
k2vett0203 = cross(k2,O2O3);
%creo l'ultima colonna del jacobiano
J3 = [k2vett0203;k2];
%creazione J2
k1 = T01(1:3,3);
O1O3 = T03(1:3,4)-T01(1:3,4);
k1vett0103 = cross(k1,O1O3);
J2 = [k1vett0103;k1];

%creazione J1
k0 = [0;0;1];
O0O3 = T03(1:3,4);
K0vettO0O3 = cross(k0,O0O3);
J1 = [K0vettO0O3;k0];

%Jacobiano totale
J = [J1,J2,J3];

%calcolo Y
dq = [dq1;dq2;dq3];
Y = jacobian(J*dq, [a1;a2;a3]);
% R = T03(1:3,1:3);
% p = T03(1:3,4);
% dp = simplify(jacobian(p,[q1,q2,q3]));
% dR = simplify(jacobian(R(:),[q1,q2,q3]));
% J = [dp;dR];