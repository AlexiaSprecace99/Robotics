clc
clear
clear all

%syms a k;

bagselect = rosbag('Elastic.bag');
j = 1;
t_end = 248.46;

% Bounds for a and k, assuming both are positive
lb = [0, 0]; % Lower bounds for a and k
ub = [inf, inf]; % Upper bounds for a and k

%Selezione Topic Desiderati
AlterEgo_State = select(bagselect,'Topic','/AlterEgoBase5/alterego_state');
Kin_Topic = select(bagselect,'Topic','/AlterEgoBase5/left/kin_des_jnt_topic');
Ref_Cubes_Topic = select(bagselect,'Topic','/AlterEgoBase5/left/ref_cubes_eq');
Q_Meas_Topic = select(bagselect,'Topic','/AlterEgoBase5/left/q_dot_meas_topic');
Theta_s_topic = select(bagselect,'Topic','/AlterEgoBase5/left/theta_s_topic');
Meas_Arm_topic = select(bagselect,'Topic','/AlterEgoBase5/left/meas_arm_shaft');
Meas_Arm_m1_topic = select(bagselect,'Topic','/AlterEgoBase5/left/meas_arm_m1');
Meas_Arm_m2_topic = select(bagselect,'Topic','/AlterEgoBase5/left/meas_arm_m2');


%Lettura dei messaggi(diventano cell)
msgs_State = readMessages(AlterEgo_State,'DataFormat','struct');
msgs_Kin = readMessages(Kin_Topic,'DataFormat','struct');
msgs_Ref_Cubes = readMessages(Ref_Cubes_Topic,'DataFormat','struct');
msgs_Q_Meas = readMessages(Q_Meas_Topic,'DataFormat','struct');
msgs_Theta_s = readMessages(Theta_s_topic,'DataFormat','struct');
msgs_arm = readMessages(Meas_Arm_topic,'DataFormat','struct');
msgs_arm_m1 = readMessages(Meas_Arm_m1_topic,'DataFormat','struct');
msgs_arm_m2 = readMessages(Meas_Arm_m2_topic,'DataFormat','struct');


%Se topic contengono più campi estraggo il campo desiderato
tau_elastica_left = cellfun(@(m) double(m.LeftQTauLinkElastic),msgs_State,'UniformOutput',false);
Q_des = cellfun(@(m) double(m.QDes),msgs_Kin,'UniformOutput',false);
Vel_des = cellfun(@(m) double(m.QdDes),msgs_Kin,'UniformOutput',false);
Acc_des = cellfun(@(m) double(m.QddDes),msgs_Kin,'UniformOutput',false);
Ref_Cubes = cellfun(@(m) double(m.Data),msgs_Ref_Cubes,'UniformOutput',false);
Q_Meas = cellfun(@(m) double(m.QDes),msgs_Q_Meas,'UniformOutput',false);
Tau_computed = cellfun(@(m) double(m.QdDes),msgs_Theta_s,'UniformOutput',false);
Errore = cellfun(@(m) double(m.QDes),msgs_Theta_s,'UniformOutput',false);
Residuo = cellfun(@(m) double(m.QddDes),msgs_Theta_s,'UniformOutput',false);
Shaft = cellfun(@(m) double(m.Data),msgs_arm,'UniformOutput',false);
M1 = cellfun(@(m) double(m.Data),msgs_arm_m1,'UniformOutput',false);
M2 = cellfun(@(m) double(m.Data),msgs_arm_m2,'UniformOutput',false);


% %Separo i valori di ogni giunto mettendoli in un vettore
% %Tau_Elastica
for i = 1:length(Residuo)
    residuo(i,1) = Residuo{i}(1);
    residuo(i,2) = Residuo{i}(2);
    residuo(i,3) = Residuo{i}(3);
    residuo(i,4) = Residuo{i}(4);
    residuo(i,5) = Residuo{i}(5);
    residuo(i,6) = Residuo{i}(6);
end
%
% %Q_des
%
for i = 1:length(Shaft)
    shaft(i,1) = Shaft{i}(1);
    shaft(i,2) = Shaft{i}(2);
    shaft(i,3) = Shaft{i}(3);
    shaft(i,4) = Shaft{i}(4);
    shaft(i,5) = Shaft{i}(5);
    shaft(i,6) = Shaft{i}(6);
    %
end

%
% %Vel_des
%
for i = 1:length(M1)
    m1(i,1) = M1{i}(1);
    m1(i,2) = M1{i}(2);
    m1(i,3) = M1{i}(3);
    m1(i,4) = M1{i}(4);
    m1(i,5) = M1{i}(5);
    m1(i,6) = M1{i}(6);
end

for i = 1:length(M2)
    m2(i,1) = M2{i}(1);
    m2(i,2) = M2{i}(2);
    m2(i,3) = M2{i}(3);
    m2(i,4) = M2{i}(4);
    m2(i,5) = M2{i}(5);
    m2(i,6) = M2{i}(6);
end

% Initial guesses for a and k
initial_guess = [9,0.008]; % Example initial guesses for a and k

% Options for the optimizer
options = optimoptions('fmincon', 'Display', 'iter-detailed', ...                          
                                'MaxFunctionEvaluations', 1e6, ...
                                'MaxIteration', 1e7);

t_end = 100;
t1 = 0:(t_end/size(shaft,1)):t_end-(t_end/size(shaft,1));

t2 = 0:(t_end/size(residuo,1)):t_end-(t_end/size(residuo,1));

residuo_interp = interp1(t2, residuo(:,5), t1, "linear");

%inizializzazione funzione
fun = @(v) my_cost(v, residuo_interp, shaft, m1, m2);
% Solving the optimization problem
coef = fmincon(fun, initial_guess, [], [], [], [], lb, ub, [], options);%,[],[],[],[],[100 100],[0,0],[],options);

%do not try this at home
% coef = fminunc(fun, initial_guess );



% for i = 1 : length(Shaft)
%     tau_elastica_44(i) = coef(2)*sinh(coef(1)*(Shaft_4(i) - M1_4(i))) +  coef(2)*sinh(coef(1)*(Shaft_4(i) - M2_4(i)));
%     %    tau_elastica_22(i) = 2*k*sinh(a*(Shaft_2(i) - ((M1_2(i) + M2_2(i))/2)))*cosh(a*(M1_2(i) - M2_2(i))/2);
% end

tau_elastica = my_tau(coef(1), coef(2), shaft(:,5), m1(:,5), m2(:,5));

tau_initial = my_tau(initial_guess(1), initial_guess(2), shaft(:,5), m1(:,5), m2(:,5));

%[best_ak, ~] = fmincon(fun, initial_guess, [], [], [], [], lb, ub, [], options);
%errore = @(a,k) norm(Residuo_2 - matlabFunction(tau_elastica_2), 2);
figure; grid on;
plot(t1, tau_elastica, 'b');
hold on; grid on;
plot(t1, residuo_interp, 'r');
plot(t1, tau_initial, 'g');
xlabel("Time [sec]");
ylabel("Tau Elastic");
title("Parameter Optimization");
legend('optimization result', 'residual', 'initial guess')

function J = my_cost(v, res, q, th1, th2)

    a = v(1);
    k = v(2);
    
    tau = my_tau(a,k, q(:,5), th1(:,5), th2(:,5));
    
    % Objective: minimize the sum of squared differences
    J = mean( abs(tau' - res).^2 );
end


function tau = my_tau(a,k, q, th1, th2)
    tau = 2 * k * sinh( a * ( q - ( ( th1 + th2 ) / 2) ) ) .* cosh( a * ( th1 - th2 ) / 2 );
    %tau = k1*sinh(a1*(q-th1)) + k2*sinh(a2*(q-th2));
end

