%Estrazione Dati dalla ROSBAG
clc
clear
clear all

bagselect = rosbag('Feedback.bag');
j = 1;
%Selezione Topic Desiderati
AlterEgo_State = select(bagselect,'Topic','/AlterEgoBase5/alterego_state');
Kin_Topic = select(bagselect,'Topic','/AlterEgoBase5/left/kin_des_jnt_topic');
Ref_Cubes_Topic = select(bagselect,'Topic','/AlterEgoBase5/left/ref_cubes_eq');
Q_Meas_Topic = select(bagselect,'Topic','/AlterEgoBase5/left/q_dot_meas_topic');
Theta_s_topic = select(bagselect,'Topic','/AlterEgoBase5/left/theta_s_topic');


%Lettura dei messaggi(diventano cell)
msgs_State = readMessages(AlterEgo_State,'DataFormat','struct');
msgs_Kin = readMessages(Kin_Topic,'DataFormat','struct');
msgs_Ref_Cubes = readMessages(Ref_Cubes_Topic,'DataFormat','struct');
msgs_Q_Meas = readMessages(Q_Meas_Topic,'DataFormat','struct');
msgs_Theta_s = readMessages(Theta_s_topic,'DataFormat','struct');


%Se topic contengono più campi estraggo il campo desiderato
tau_elastica_left = cellfun(@(m) double(m.LeftQTauLinkElastic),msgs_State,'UniformOutput',false);
Q_des = cellfun(@(m) double(m.QDes),msgs_Kin,'UniformOutput',false);
Vel_des = cellfun(@(m) double(m.QdDes),msgs_Kin,'UniformOutput',false);
Acc_des = cellfun(@(m) double(m.QddDes),msgs_Kin,'UniformOutput',false);
Ref_Cubes = cellfun(@(m) double(m.Data),msgs_Ref_Cubes,'UniformOutput',false);
Q_Meas = cellfun(@(m) double(m.QDes),msgs_Q_Meas,'UniformOutput',false);
Tau_computed = cellfun(@(m) double(m.QdDes),msgs_Theta_s,'UniformOutput',false);
Errore = cellfun(@(m) double(m.QDes),msgs_Theta_s,'UniformOutput',false);


%Separo i valori di ogni giunto mettendoli in un vettore
%Tau_Elastica
for i = 1:length(tau_elastica_left)
tau_elastica_left_1(i) = tau_elastica_left{i}(1);
tau_elastica_left_2(i) = tau_elastica_left{i}(2);
tau_elastica_left_3(i) = tau_elastica_left{i}(3);
tau_elastica_left_4(i) = tau_elastica_left{i}(4);
tau_elastica_left_5(i) = tau_elastica_left{i}(5);
tau_elastica_left_6(i) = tau_elastica_left{i}(6);
end

%Q_des

for i = 2:length(Q_des)
Q_left_1(j) = Q_des{i}(1);
Q_left_2(j) = Q_des{i}(2);
Q_left_3(j) = Q_des{i}(3);
Q_left_4(j) = Q_des{i}(4);
Q_left_5(j) = Q_des{i}(5);
Q_left_6(j) = Q_des{i}(6);
j = j + 1;
end


%Vel_des

for i = 1:length(Vel_des)
Vel_des_left_1(i) = Vel_des{i}(1);
Vel_des_left_2(i) = Vel_des{i}(2);
Vel_des_left_3(i) = Vel_des{i}(3);
Vel_des_left_4(i) = Vel_des{i}(4);
Vel_des_left_5(i) = Vel_des{i}(5);
Vel_des_left_6(i) = Vel_des{i}(6);
end

%Acc_des

for i = 1:length(Acc_des)
Acc_des_left_1(i) = Acc_des{i}(1);
Acc_des_left_2(i) = Acc_des{i}(2);
Acc_des_left_3(i) = Acc_des{i}(3);
Acc_des_left_4(i) = Acc_des{i}(4);
Acc_des_left_5(i) = Acc_des{i}(5);
Acc_des_left_6(i) = Acc_des{i}(6);
end

%Ref_Cubes

for i = 1:length(Ref_Cubes)
Ref_Cubes_left_1(i) = Ref_Cubes{i}(1);
Ref_Cubes_left_2(i) = Ref_Cubes{i}(2);
Ref_Cubes_left_3(i) = Ref_Cubes{i}(3);
Ref_Cubes_left_4(i) = Ref_Cubes{i}(4);
Ref_Cubes_left_5(i) = Ref_Cubes{i}(5);
Ref_Cubes_left_6(i) = Ref_Cubes{i}(6);
end

%Q_meas

for i = 1:length(Q_Meas)
Q_Meas_left_1(i) = Q_Meas{i}(1);
Q_Meas_left_2(i) = Q_Meas{i}(2);
Q_Meas_left_3(i) = Q_Meas{i}(3);
Q_Meas_left_4(i) = Q_Meas{i}(4);
Q_Meas_left_5(i) = Q_Meas{i}(5);
Q_Meas_left_6(i) = Q_Meas{i}(6);
end

%Tau_computed

for i = 1:length(Tau_computed)
Tau_computed_left_1(i) = Tau_computed{i}(1);
Tau_computed_left_2(i) = Tau_computed{i}(2);
Tau_computed_left_3(i) = Tau_computed{i}(3);
Tau_computed_left_4(i) = Tau_computed{i}(4);
Tau_computed_left_5(i) = Tau_computed{i}(5);
Tau_computed_left_6(i) = Tau_computed{i}(6);
end

%Errore

for i = 1:length(Errore)
Errore_left_1(i) = Errore{i}(1);
Errore_left_2(i) = Errore{i}(2);
Errore_left_3(i) = Errore{i}(3);
Errore_left_4(i) = Errore{i}(4);
Errore_left_5(i) = Errore{i}(5);
Errore_left_6(i) = Errore{i}(6);
end

t = 0:0.0193700787401575:248.46;
figure(1);
plot(t,Q_Meas_left_1,'r');
hold on;
plot(t,Q_left_1,'b');
hold on;
plot(t,Ref_Cubes_left_1,'g');

xlabel('Time[s]');
ylabel('Joint position[rad]');
title('Joint 1');

legend('q misurata','q desiderata', 'q comandata');

figure(2);
plot(t,Q_Meas_left_2,'r');
hold on;
plot(t,Q_left_2,'b');
hold on;
plot(t,Ref_Cubes_left_2,'g');

xlabel('Time[s]');
ylabel('Joint position[rad]');
title('Joint 2');

legend('q misurata','q desiderata', 'q comandata');

figure(3);
plot(t,Q_Meas_left_3,'r');
hold on;
plot(t,Q_left_3,'b');
hold on;
plot(t,Ref_Cubes_left_3,'g');

xlabel('Time[s]');
ylabel('Joint position[rad]');
title('Joint 3');

legend('q misurata','q desiderata', 'q comandata');

figure(4);
plot(t,Q_Meas_left_4,'r');
hold on;
plot(t,Q_left_4,'b');
hold on;
plot(t,Ref_Cubes_left_4,'g');

xlabel('Time[s]');
ylabel('Joint position[rad]');
title('Joint 4');

legend('q misurata','q desiderata', 'q comandata');

figure(5);
plot(t,Q_Meas_left_5,'r');
hold on;
plot(t,Q_left_5,'b');
hold on;
plot(t,Ref_Cubes_left_5,'g');

xlabel('Time[s]');
ylabel('Joint position[rad]');
title('Joint 5');

legend('q misurata','q desiderata', 'q comandata');

figure(6);
plot(t,Q_Meas_left_6,'r');
hold on;
plot(t,Q_left_6,'b');
hold on;
plot(t,Ref_Cubes_left_6,'g');

xlabel('Time[s]');
ylabel('Joint position[rad]');
title('Joint 6');

legend('q misurata','q desiderata', 'q comandata');




