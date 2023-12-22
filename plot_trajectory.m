%% PLOTS

close all;
clear L;
f = figure;
f.WindowState = 'maximized';
pause(0.1)
a1 = 1;
a2 = 1;

% Getting Vectors from Sim
ex = out.ex.Data;
ey = out.ey.Data;
for i = 1:1:size(ex,3)
    err_x(i) = ex(1,1,i);
    err_y(i) = ey(1,1,i);
end
q_out = out.x_state.Data(1:2,1,:);
xi_des_out = out.xi_des.Data.';
t_out = out.tout.';

% post processing
res = 0.1;
[t_sim, q_sim] = adjust_time(t_out,q_out,res);

% errors
h(1) = subplot(1,2,1);
hold on; grid on;
plot(t_out,err_x);
plot(t_out,err_y);
xlabel("Time[s]");
ylabel("Errors[m]");
legend('$e_x$','$e_y$','Interpreter','latex')
title('Errors')

% robot motion
h(2) = subplot(1,2,2);
hold on
axis equal
axis ([-0.5 2.5 -1.0 2.0])
grid on
title('Animation - Feedback Linearization')

% initialize animation
L(1) = plot(0,0,'-ko','linewidth',2);
t_h = text(2.2,2.2,['(' num2str(0) ')']);
t_xi = plot(xi_des_out(1,:),xi_des_out(2,:),'r');

for i=1:1:size(t_sim,2)
    
    delete(L(1))
    delete(t_h)
    
    t_h = text(2.2,2.2,['(' num2str(t_sim(i)) ')']);
    L(1)=plot_robot(q_sim(:,i),1);
    legend(L, {'execution'})
    
    drawnow
    
end