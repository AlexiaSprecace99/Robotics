% Parametri dell'ellisse
a = 0.5;  % Semiasse maggiore lungo l'asse x
b = 0.7;  % Semiasse minore lungo l'asse y
c = 1;  % Semiasse lungo l'asse z
numPoints = 100;  % Numero di punti lungo la traiettoria

% Calcola gli angoli dei punti lungo la traiettoria
theta = linspace(0, 2*pi, numPoints);

% Calcola le coordinate dei punti lungo l'ellisse
x = a * cos(theta);
y = b * sin(theta);
z = c * sin(theta);

% Crea una figura 3D
figure;
plot3(x, y, z, 'b', 'LineWidth', 2);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Traiettoria Ciclica in 3D');

% Aggiungi i punti iniziali e finali alla traiettoria
hold on;
plot3(x(1), y(1), z(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(x(end), y(end), z(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
hold off;

% Imposta gli assi in modo da avere lo stesso scaling
axis equal;
% te=30; %Final time
% position = [];
% velocity = [];
% x_start=[1 1 1]'; %Starting position
% v_start=[0 0 0]'; %Starting velocity
% a_start=[0 0 0]'; %Starting acceleration
% 
% %x_end=[100 45 20]'; %Final position
% %v_end=[1 5 0]'; %Final velocity
% x_end=[1.5 1.5 0.5]'; %Final position
% v_end = [5 5 5]'; %Final velocity
% a_end=[0 0 0]'; %Final acceleration
% 
% A=[te^3 te^4 te^5;
%    3*te^2 4*te^3 5*te^4;
%    6*te 12*te^2 20*te^3];
% 
% PR=zeros(6,3); %Each column is an [a0 a1 a2 a3 a4 a5]' vector
% 
% %The first three row of P comes from boundary condition (a0,a1,a2)
% PR(1,:)=[x_start(1),x_start(2),x_start(3)];
% PR(2,:)=[v_start(1),v_start(2),v_start(3)];
% PR(3,:)=[a_start(1)/2,a_start(2)/2,a_start(3)/2];
% 
% for i=1:3
% %when i = 1 we only consider x coordinate, when i = 2 we consider y
% %coordinate
% x_tend = x_end(i);
% v_tend = v_end(i);
% a_tend = a_end(i);
% M=zeros(3,1);
% M(1) = x_tend - PR(1,i) - PR(2,i)*te - PR(3,i)*(te^2);
% M(2) = v_tend - PR(2,i) - 2*PR(3,i)*te;
% M(3) = a_tend - 2*PR(3,i);
% 
% PR(4:6,i) = inv(A)*M; %the first three row of P have already been computed 
% 
% end
% i = 1;
% for t= 0:0.1:te
% x = x_start(1) + PR(2,1)*t + PR(3,1)*t^2 + PR(4,1)*t^3 + PR(5,1)*t^4 + PR(6,1)*t^5;
% y = x_start(2) + PR(2,2)*t + PR(3,2)*t^2 + PR(4,2)*t^3 + PR(5,2)*t^4 + PR(6,2)*t^5;
% z = x_start(3) + PR(2,3)*t + PR(3,3)*t^2 + PR(4,3)*t^3 + PR(5,3)*t^4 + PR(6,3)*t^5;
% 
% vx = PR(2,1) + 2*PR(3,1)*t+3*PR(4,1)*t^2 + 4*PR(5,1)*t^3 + 5*PR(6,1)*t^4;
% vy = PR(2,2) + 2*PR(3,2)*t+3*PR(4,2)*t^2 + 4*PR(5,2)*t^3 + 5*PR(6,2)*t^4;
% vz = PR(2,3) + 2*PR(3,3)*t+3*PR(4,3)*t^2 + 4*PR(5,3)*t^3 + 5*PR(6,3)*t^4;
% 
% 
% position(:,i) = [x y z]';
% velocity(:,i) = [vx vy vz]';
% 
% i = i+1;
% end
% p = position;
% dp = velocity; 