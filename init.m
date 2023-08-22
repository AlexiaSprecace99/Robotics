clear all
close all
clc

% real parameters
a1 = 1;
a2 = 1;
a3 = 1;

g = 9.81;
Lamda = 10*eye(3);
Ks = 50*eye(3);

% initialize jointstate
q_0 = [1.3 ; -1.2; 1.1];


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

rel_err = 0.5;
