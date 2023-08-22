%% PLOTS
close all

dt  =  0.2;

% Create a blank figure
f = figure;
f.WindowState = 'maximized';

% Getting Vectors from Sim
e_out  =  out.e_out.Data.';
q_out  =  out.q_out.Data.';
x_des  =  out.xi_des_out.Data.';
tout  =  out.tout.';

% Post processing data
time  =  0 : dt : tout(end);
e_pp  =  zeros(size(e_out,1),size(time,2));
q_pp  =  zeros(size(q_out,1),size(time,2));
x_des_pp  =  zeros(size(x_des,1),size(time,2));

for i  =  1 : size(time,2)
    [d, ix]  =  min(abs(tout-time(i)));
    e_pp(:,i)  =  e_out(:, ix);
    q_pp(:,i)  =  q_out(:, ix);
    x_des_pp(:,i)  =  x_des(:, ix);
end

% errors
h(1) = subplot(2,2,1);
hold on
grid on
plot(time, e_pp(1,:));
plot(time, e_pp(2,:));
plot(time, e_pp(3,:));
legend('e_1','e_2','e_3');
title('errors')
drawnow

% % parameters
% h(2) = subplot(2,2,3);
% hold on
% plot(time, a1*ones(1,size(PI_pp,2)));
% plot(time, a2*ones(1,size(PI_pp,2)));
% plot(time, PI_pp(1,:));
% plot(time, PI_pp(2,:));
% legend('a_1','a_2','a_{est_1}','a_{est_2}')
% title('params')
% drawnow

% robot motion
h(2) = subplot(2,2,2);
grid on
axis equal
axis ([-0.5 2.5 -0.5 2.5])
grid on
title('robot motion')

L(1) = plot3(0,0,0,'-ko','linewidth',2);
t_h = text(1.8,2.2,[num2str(0) ' s ( step ' num2str(0) ')']);
for i = 1:1:size(q_pp,2)
    
    delete(L(1))
    delete(t_h)
    
    t_h = text(1.8,2.2,[num2str(time(i)) ' s ( step ' num2str(i) ')']);
    [p0,p1,p2,p3] = direct_kinematics(q_pp(:,i));
    plot3(x_des_pp(1,:),x_des_pp(2,:),x_des_pp(3,:),'r'); hold on; grid on;
    %direct kinematics
    L(1) = plot3([p0(1) p1(1) p2(1) p3(1)],[p0(2) p1(2) p2(2) p3(2)],[p0(3) p1(3) p2(3) p3(3)],'-ko','linewidth',2);
    legend(L, {'execution'})
   
    
    
    drawnow

end