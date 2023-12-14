%syms  theta1(t) theta2(t) k t q1(t) q2(t) dq1(t) dq2(t) ddq1(t) ddq2(t) dtheta1(t) dtheta2(t) G1 G2 B11 B12 B21 B22 C12 C11 C21 C22 B11C11 B12C21 B11C12 B12C22 B21C11 B22C21 B21C12 B22C22 B21G1 B22G2;
syms theta1(t) theta2(t) dq1(t) dq2(t) q1(t) q2(t) k dtheta1(t) dtheta2(t);
Q_ddot = sym('q_ddot',[2 1],'real');
B = sym('b',[2 2],'real'); %intendo gia l'inversa
C = sym('c',[2 2],'real');
Theta = sym('theta',[2 1],'real');
Q = sym('q',[2 1],'real');
DQ = sym('dq',[2 1],'real');
G = sym('g',[2 1],'real');
q(t) = [q1(t);q2(t)];
dq(t) = [dq1(t);dq2(t)];
%dq = [dq1;dq2];
theta(t) = [theta1(t);theta2(t)];

%acc = k*theta(t)-k*q(t)-inv(M)*C*dq(t)-inv(M)*G;
% q1_ddot = acc(1);
% q2_ddot = acc(2);

%B11: elemento 11 inverso di B
%B12: elemento 12 inverso di B

%q1_ddot = B11*k*(theta1(t)-q1(t))+B12*k*(theta2(t)-q2(t)) + (B11C11 + B12C21)*dq1 + (B11C12+B12C22)*dq2 + B11G1 + B12G2;
%q2_ddot = B21*k*(theta1(t)-q1(t))+B22*k*(theta2(t)-q2(t)) + (B21C11 + B22C21)*dq1 + (B21C12+B22C22)*dq2 + B21G1 + B22G2;

Q_ddot = B*k*(theta(t)-q(t))- B*C*dq(t) - B*G;

%differenziazione coordinata x
x(t) = -sin(q1(t))*dq1(t) - sin(q1(t)+q2(t))*(dq1(t)+dq2(t)); %derivata prima
x_dot(t) = diff(x(t),t); %derivata seconda
x_dot(t) = subs(x_dot(t), diff(dq1(t), t), Q_ddot(1));
x_dot(t) = subs(x_dot(t), diff(dq2(t), t), Q_ddot(2));
x_dot(t) = subs(x_dot(t), diff(q1(t), t), dq1(t));
x_dot(t) = subs(x_dot(t), diff(q2(t), t), dq2(t));
x_ddot(t) = diff(x_dot(t),t); %derivata terza
x_ddot(t) = subs(x_ddot(t), diff(dq1(t), t), Q_ddot(1));
x_ddot(t) = subs(x_ddot(t), diff(dq2(t), t), Q_ddot(2));
x_ddot(t) = subs(x_ddot(t), diff(q1(t), t), dq1(t));
x_ddot(t) = subs(x_ddot(t), diff(q2(t), t), dq2(t));
x_ddot(t) = subs(x_ddot(t), diff(theta1(t), t), dtheta1(t));
x_ddot(t) = subs(x_ddot(t), diff(theta2(t), t), dtheta2(t));
%x_dtheta1 = -k*sin(q1(t)+q2(t)) - k*sin(q1(t));
%x_dtheta2 = -k*sin(q1(t)+q2(t));
x_dtheta1_intero = coeffs(x_ddot(t), dtheta1(t));
x_dtheta1 = x_dtheta1_intero(2);
x_dtheta2_intero = coeffs(x_ddot(t), dtheta2(t));
x_dtheta2 = x_dtheta2_intero(2);
f_x = x_ddot(t) - x_dtheta1*dtheta1(t) - x_dtheta2*dtheta2(t);
%f_x = x_ddot(t) - x_dtheta1*dtheta1(t) - x_dtheta2*dtheta2(t);

%differenziazione coordinata y
y(t) = cos(q1(t))*dq1(t) + cos(q1(t)+q2(t))*(dq1(t)+dq2(t));
y_dot(t) = diff(y(t),t);
y_dot(t) = subs(y_dot(t), diff(dq1(t), t), Q_ddot(1));
y_dot(t) = subs(y_dot(t), diff(dq2(t), t), Q_ddot(2));
y_dot(t) = subs(y_dot(t), diff(q1(t), t), dq1(t));
y_dot(t) = subs(y_dot(t), diff(q2(t), t), dq2(t));
y_ddot(t) = diff(y_dot(t),t);
y_ddot(t) = subs(y_ddot(t), diff(dq1(t), t), Q_ddot(1));
y_ddot(t) = subs(y_ddot(t), diff(dq2(t), t), Q_ddot(2));
y_ddot(t) = subs(y_ddot(t), diff(q1(t), t), dq1(t));
y_ddot(t) = subs(y_ddot(t), diff(q2(t), t), dq2(t));
y_ddot(t) = subs(y_ddot(t), diff(theta1(t), t), dtheta1(t));
y_ddot(t) = subs(y_ddot(t), diff(theta2(t), t), dtheta2(t));

y_dtheta1_intero = coeffs(y_ddot(t), dtheta1(t));
y_dtheta1 = y_dtheta1_intero(2);
y_dtheta2_intero = coeffs(y_ddot(t), dtheta2(t));
y_dtheta2 = y_dtheta2_intero(2);
f_y = y_ddot(t) - y_dtheta1*dtheta1 - y_dtheta2*dtheta2;

coeff_theta1 = coeffs(x_ddot(t),dtheta1(t));
E = [x_dtheta1 x_dtheta2; y_dtheta1 y_dtheta2];
F = [f_x;f_y];
Y = [x_ddot(t);y_ddot(t)];
U = -inv(E)*F + inv(E)*Y;
% acc = k*theta2-k*q-inv(M)*C*dq-inv(M)*G;
% f = [x3;x4;acc(1);acc(2);0;0];
% g = [0 0; 0 0; 0 0; 0 0;1 0;0 1];

%x_dot = diff(x,q(t));
%y_dot = jacobian(y,q);

