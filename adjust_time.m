function [t_sim,q_sim] = adjust_time(t_in,q_in,res)
% ADJUST TIME Adjusts time vector in order to have a realistic animation in
% time

% Creating good time vector
t_f = t_in(end);
t_0 = t_in(1);
t_sim = t_0:res:t_f;

% Getting the nearest values in q_sim
q_sim = zeros(size(q_in,1), size(t_sim,2)); % preallocating
for i = 1:size(t_sim,2)
    [~, index] = min(abs(t_in - t_sim(i))); % getting index of nearst element
    q_sim(:,i) = q_in(:,index);
end

end