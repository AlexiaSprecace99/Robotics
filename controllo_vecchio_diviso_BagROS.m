%Estrazione Dati dalla ROSBAG
clc
clear
clear all

bagselect = rosbag('controllo_vecchio_diviso.bag');
j = 1;

%Selezione Topic Desiderati
AlterEgo_State = select(bagselect,'Topic','/AlterEgoBase5/alterego_state');
Kin_Topic = select(bagselect,'Topic','/AlterEgoBase5/left/kin_des_jnt_topic');
Ref_Cubes_Topic = select(bagselect,'Topic','/AlterEgoBase5/left/ref_cubes_eq');
%Q_Meas_Topic = select(bagselect,'Topic','/AlterEgoBase5/left/q_dot_meas_topic');
%Theta_s_topic = select(bagselect,'Topic','/AlterEgoBase5/left/theta_s_topic');

%Lettura dei messaggi(diventano cell)
msgs_State = readMessages(AlterEgo_State,'DataFormat','struct');
msgs_Kin = readMessages(Kin_Topic,'DataFormat','struct');
msgs_Ref_Cubes = readMessages(Ref_Cubes_Topic,'DataFormat','struct');
%msgs_Q_Meas = readMessages(Q_Meas_Topic,'DataFormat','struct');
%msgs_Theta_s = readMessages(Theta_s_topic,'DataFormat','struct');

%Se topic contengono più campi estraggo il campo desiderato
%tau_elastica_left = cellfun(@(m) double(m.LeftQTauLinkElastic),msgs_State,'UniformOutput',false);
Q_Meas = cellfun(@(m) double(m.LeftMeasArmShaft),msgs_State,'UniformOutput',false);
Q_des = cellfun(@(m) double(m.QDes),msgs_Kin,'UniformOutput',false);
%Vel_des = cellfun(@(m) double(m.QdDes),msgs_Kin,'UniformOutput',false);
%Acc_des = cellfun(@(m) double(m.QddDes),msgs_Kin,'UniformOutput',false);
Ref_Cubes = cellfun(@(m) double(m.Data),msgs_Ref_Cubes,'UniformOutput',false);
%Q_Meas = cellfun(@(m) double(m.QDes),msgs_Q_Meas,'UniformOutput',false);
%Tau_computed = cellfun(@(m) double(m.QdDes),msgs_Theta_s,'UniformOutput',false);
%Errore = cellfun(@(m) double(m.QDes),msgs_Theta_s,'UniformOutput',false);

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
j = 1;
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


%t = 0:0.01:97.22;
t = 0:0.0255538414069732:248.46;
%t1 = 0:0.0025078677:97.22;
t1 = 0:0.0064090592514252:248.46;
Q_meas_left_1_interp = interp1(t1,Q_Meas_left_1,t,'linear');
Q_meas_left_2_interp = interp1(t1,Q_Meas_left_2,t,'linear');
Q_meas_left_3_interp = interp1(t1,Q_Meas_left_3,t,'linear');
Q_meas_left_4_interp = interp1(t1,Q_Meas_left_4,t,'linear');
Q_meas_left_5_interp = interp1(t1,Q_Meas_left_5,t,'linear');
Q_meas_left_6_interp = interp1(t1,Q_Meas_left_6,t,'linear');

figure(1); grid on;
plot(t,Q_meas_left_1_interp,'r');
hold on; grid on;
plot(t,Q_left_1,'b');
% hold on; grid on;
% plot(t,Ref_Cubes_left_1,'g');

xlabel('Time[s]');
ylabel('Joint position[rad]');
title('Joint 1');

legend('q misurata','q desiderata', 'q comandata');

figure(2); grid on;
plot(t,Q_meas_left_2_interp,'r');
hold on; grid on;
plot(t,Q_left_2,'b');
hold on; grid on;
plot(t,Ref_Cubes_left_2,'g');

xlabel('Time[s]');
ylabel('Joint position[rad]');
title('Joint 2');

legend('q misurata','q desiderata', 'q comandata');

figure(3); grid on;
plot(t,Q_meas_left_3_interp,'r');
hold on; grid on;
plot(t,Q_left_3,'b');
hold on; grid on;
plot(t,Ref_Cubes_left_3,'g');

xlabel('Time[s]');
ylabel('Joint position[rad]');
title('Joint 3');

legend('q misurata','q desiderata', 'q comandata');

figure(4); grid on;
plot(t,Q_meas_left_4_interp,'r');
hold on; grid on;
plot(t,Q_left_4,'b');
hold on; grid on;
plot(t,Ref_Cubes_left_4,'g');

xlabel('Time[s]');
ylabel('Joint position[rad]');
title('Joint 4');

legend('q misurata','q desiderata', 'q comandata');

figure(5); grid on;
plot(t,Q_meas_left_5_interp,'r');
hold on; grid on;
plot(t,Q_left_5,'b');
hold on; grid on;
plot(t,Ref_Cubes_left_5,'g');

xlabel('Time[s]');
ylabel('Joint position[rad]');
title('Joint 5');

legend('q misurata','q desiderata', 'q comandata');

figure(6); grid on;
plot(t,Q_meas_left_6_interp,'r');
hold on; grid on;
plot(t,Q_left_6,'b');
hold on; grid on;
plot(t,Ref_Cubes_left_6,'g');

xlabel('Time[s]');
ylabel('Joint position[rad]');
title('Joint 6');

legend('q misurata','q desiderata', 'q comandata');
