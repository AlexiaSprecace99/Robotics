close all;
f = figure;
f.WindowState = 'maximized';
pause(0.1)

% Getting Vectors from Sim
e_out = out.e_out.Data.';
q_out = out.q_out1.Data.';
xi_des_out = out.xi_des_out.Data.';
t_out = out.tout.';

% post processing
res = 0.1;
[t_sim, q_sim] = adjust_time(t_out,q_out,res);

% errors
figure(1);
plot(t_out,e_out(1,:)); hold on;
plot(t_out,e_out(2,:)); hold on;
plot(t_out,e_out(3,:)); grid on;
legend('$e_1$','$e_2$','$e_3$','Interpreter','latex');
title('Errors');


% initialize animation
L(1) = plot3(0,0,0,'-ko','linewidth',2);
t_h = text(2.2,2.2,['(' num2str(0) ')']);
%t_xi = plot(xi_des_out(1,:),xi_des_out(2,:),xi_des_out(3,:),'r');

for i=1:1:size(t_sim,2)
    % robot motion
    axis equal
    axis ([-1.5 2.5 -1.5 1.5]);
    title('Animation - Computed Torque');

    delete(L(1))
    delete(t_h)
    
    t_h = text(2.2,2.2,['(' num2str(t_sim(i)) ')']);
    [p0,p1,p2,p3] = direct_kinematics(q_sim(:,i));
    plot3(xi_des_out(1,:),xi_des_out(2,:),xi_des_out(3,:),'r'); hold on; grid on;
    %direct kinematics
    L(1) = plot3([p0(1) p1(1) p2(1) p3(1)],[p0(2) p1(2) p2(2) p3(2)],[p0(3) p1(3) p2(3) p3(3)],'-ko','linewidth',2);
    legend(L, {'Robot animation'})
    
    drawnow
    
end