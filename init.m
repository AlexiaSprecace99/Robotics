clear all
close all
clc

% real parameters
a1 = 1;
a2 = 1;
a3 = 1;

% initial estimates
initial_estimated_params = [1.5; 1.5; 1.5];

% initialize jointstate
q_0 = [1.3 ; -1.2; 1.1];

% control gains (suggested sim time = 60)
Ke = 1; % 1 / 1000;
Kk = eye(3); % 0.1 / 1