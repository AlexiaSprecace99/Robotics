k = 1;
a1 = 1;
a2 = 1;
theta1 = sym('theta1','real');
theta2 = sym('theta2','real');
dq1 = sym('dq1','real');
dq2 = sym('dq2','real');
q1 = sym('q1','real');
q2 = sym('q2','real');
Q_ddot = sym('q_ddot',[2 1],'real');
ad_fng_1 = sym('ad_fng_1','real');
ad_fng_2 = sym('ad_fng_2','real');

x = [q1;q2;dq1;dq2;theta1;theta2]; %Vettore di stato

q = [q1;q2];
dq = [dq1;dq2];
theta = [theta1;theta2];

B = RR_mass_matrix(q);
C = RR_coriolis_matrix(q,dq);
G = RR_gravity_vector(q);

Q_ddot = inv(B)*k*(theta-q)- inv(B)*C*dq - inv(B)*G;

f = [dq1;dq2;Q_ddot(1);Q_ddot(2);0;0];

g1 = sym([0 0 0 0 1 0]');
g2 = sym([0 0 0 0 0 1]');
g = [g1 g2];

n = 2;

% Call the Lie Bracket Function
%[g1,g2] nulla
ad_fng_1=liebracket(f,g1,x,n);
ad_fng_2=liebracket(f,g2,x,n);

A = [ad_fng_1 ad_fng_2];
det_A = det(A);
zero_det_ragg = solve(det_A == 0, [q1,q2]);



%Osservabilità
y1 = a1*cos(q1) + a2*cos(q1+q2);
y2 = a1*sin(q1) + a2*sin(q1+q2);

Lf_y1_0 = jacobian(y1,x);
Lf_y2_0 = jacobian(y2,x);
Lf_y1_1 = jacobian(Lf_y1_0*f,x);
Lf_y2_1 = jacobian(Lf_y2_0*f,x);

%nulle
% Lg1_y1_1 = jacobian(Lf_y1_0*g1,x); 
% Lg2_y1_1 = jacobian(Lf_y1_0*g2,x); 
% Lg1_y2_1 = jacobian(Lf_y2_0*g1,x); 
% Lg2_y2_1 = jacobian(Lf_y2_0*g2,x);

Lf_y1_2 = jacobian(Lf_y1_1*f,x);
Lf_y2_2 = jacobian(Lf_y2_1*f,x);

%nulle
% Lg1_y1_2 = jacobian(Lf_y1_1*g1,x); 
% Lg2_y1_2 = jacobian(Lf_y1_1*g2,x); 
% Lg1_y2_2 = jacobian(Lf_y2_1*g1,x); 
% Lg2_y2_2 = jacobian(Lf_y2_1*g2,x);

Lf_y1_3 = jacobian(Lf_y1_2*f,x);
Lf_y2_3 = jacobian(Lf_y2_2*f,x);

Lg1_y1_3 = jacobian(Lf_y1_2*g1,x); 
Lg2_y1_3 = jacobian(Lf_y1_2*g2,x); 
Lg1_y2_3 = jacobian(Lf_y2_2*g1,x); 
Lg2_y2_3 = jacobian(Lf_y2_2*g2,x);

Lf_y1_4 = jacobian(Lf_y1_3*f,x);
Lf_y2_4 = jacobian(Lf_y2_3*f,x);

O = [Lf_y1_0; Lf_y2_0; Lf_y1_1; Lf_y2_1; Lf_y1_2; Lf_y2_2;Lf_y1_3;Lf_y1_4]; % rango 6 
sottomatrice_O = [Lf_y2_0;Lf_y2_1; Lf_y1_2; Lf_y2_2; Lf_y1_3;Lf_y1_4];
%det_O = det(sottomatrice_O);
%zero_det_oss_O  = solve(det_O == 0, [q1,q2,dq1,dq2]);





% Inizializzazione di una variabile per memorizzare i determinanti
% determinanti = sym('det',[3 1],'real');
% 
% % Calcolo del determinante per ogni sottomatrice 6x6
% for i = 1:size(O, 2) - 3
%     sottomatrice4x4 = O(:, i:i+3);
%     determinante_sottomatrice = det(sottomatrice4x4);
%     determinanti(i) = determinante_sottomatrice;
% end
% zero_det_oss_1 = solve(determinanti(1) == 0, [q1,q2]);
% zero_det_oss_2 = solve(determinanti(2) == 0, [q1,q2,dq1,dq2]);
% zero_det_oss_3 = solve(determinanti(3) == 0, [q1,q2,dq1,dq2]);



