%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Project: Geometric tracking controller for quadrotor UAV on SE(3)      %
% Author: Bin Zhang                                                      %
% Date: Dec. 22nd, 2020                                                  %
% Reference: Taeyoung Lee, Melvin Leok, N. Harris McClamroch. Geometric  %
%            Tracking Control of a Quadrotor UAV on SE(3)[C]. In Proce-  %
%            edings of 49th IEEE Conference on Decision and Control,     %
%            USA: Atlanta, GA, 2010.                                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clear; clc;

%% Parameters
% UAV
param.J = diag([0.0820, 0.0845, 0.1377]);
param.m = 4.34;
param.g = 9.81;
param.d = 0.315;
param.c = 8.004e-4;
param.e3 = [0, 0, 1]';
param.allocation = [       1,        1,        1,       1
                           0, -param.d,        0, param.d
                     param.d,        0, -param.d,       0
                    -param.c,  param.c, -param.c, param.c];

% Controller
k.x = 16*param.m;
k.v = 5.6*param.m;
k.R = 8.81;
k.W = 2.54;

% Initial setup
flag = 1; % change flag for different cases, 1 for case 1, 2 for case 2

init.x = zeros(3, 1);
init.v = zeros(3, 1);
init.W = zeros(3, 1);

switch(flag)
    case 1
        init.R = eye(3);
        
    case 2        
        init.R = [1,       0,       0
                  0, -0.9995, -0.0314
                  0,  0.0314, -0.9995];        
        
end

%% Simulation
% Simulation time
t = 0:0.01:10;

% Initial condition
x0 = [init.x; init.v; reshape(init.R, 9, 1); init.W];

% Control
[t, X] = ode45(@(t, ref)Dynamics(t, ref, flag, k, param), t, x0,...
               odeset('RelTol', 1e-6, 'AbsTol', 1e-6));

%% Result processing
% Initial the data arrays
N = size(t, 1);
ref.x = zeros(3, N);
ref.v = zeros(3, N);
ref.W = zeros(3, N);
errors.x = zeros(3, N);
errors.v = zeros(3, N);
errors.R = zeros(3, N);
errors.W = zeros(3, N);
errors.Phi = zeros(1, N);
ctrl.f = zeros(1, N);
ctrl.M = zeros(3, N);
ctrl.thrust = zeros(4, N);

% Recording
for i=1:N
    
    % Recording reference trajectory
    [Xd, Bd] = reference(t(i), flag);
    ref.x(:, i) = Xd(:, 1);
    ref.v(:, i) = Xd(:, 2);
    
    % Recording errors and control inputs
    R = reshape(X(i, 7:15), 3, 3);
    f = posCtrl(k.x, k.v, param.m, param.g, param.e3,...
                R, X(i, 1:3)', X(i, 4:6)', Xd);
    [error, Phi, Wd, M] = mntCtrl(k, param, X(i, :)', Xd, Bd);
    
    ref.W(:, i) = Wd;
    
    errors.x(:, i) = error(:, 1);
    errors.v(:, i) = error(:, 2);
    errors.R(:, i) = error(:, 3);
    errors.W(:, i) = error(:, 4);
    errors.Phi(i) = Phi;
    
    ctrl.f(i) = f;
    ctrl.M(:, i) = M';
    input = [f; M];
    ctrl.thrust(:, i) = param.allocation\input;
    
end

% Plotting
figure(1)
if flag==2
    lineType = 'r*';
else
    lineType = 'r';
end
plot3(ref.x(1,:), ref.x(2,:), ref.x(3,:), lineType, 'LineWidth', 1); hold on;
plot3(X(:,1), X(:,2), X(:,3), 'k', 'LineWidth', 1);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Tracking Result(3D)')
% axis([-0.01, 0.01, -0.2, 0.2, -0.2, 0.6]);
legend('Reference Trajectory', 'Actual Trajectory', 'Location', 'Best');

figure(2)
subplot(311)
plot(t, ref.x(1,:), 'r', 'LineWidth', 1); hold on;
plot(t, X(:,1), 'k--', 'LineWidth', 1);
% xlabel('time(s)');
ylabel('X');
% axis([0, 10, -0.2, 4]);
legend('Reference Trajectory', 'Actual Trajectory', 'Location', 'Best');
title('Tracking Result(2D)');
subplot(312)
plot(t, ref.x(2,:), 'r', 'LineWidth', 1); hold on;
plot(t, X(:,2), 'k--', 'LineWidth', 1);
% xlabel('time(s)'); 
ylabel('Y');
% axis([0, 10, -1, 1]);
subplot(313)
plot(t, ref.x(3,:), 'r', 'LineWidth', 1); hold on;
plot(t, X(:,3), 'k--', 'LineWidth', 1);
xlabel('time(s)'); ylabel('Z');
% axis([0, 10, -1, 1]);

figure(3)
subplot(311)
plot(t, ref.W(1,:), 'r', 'LineWidth', 1); hold on;
plot(t, X(:,16), 'k--', 'LineWidth', 1);
% xlabel('time(s)'); 
ylabel('$\boldmath{\Omega}_x$', 'interpreter', 'latex');
% axis([0, 10, -20, 10]);
legend('Reference', 'Actual', 'Location', 'Best');
title('Angular Velocity');
subplot(312)
plot(t, ref.W(2,:), 'r', 'LineWidth', 1); hold on;
plot(t, X(:,17), 'k--', 'LineWidth', 1);
% xlabel('time(s)'); 
ylabel('$\boldmath{\Omega}_y$', 'interpreter', 'latex');
% axis([0, 10, -1, 1]);
subplot(313)
plot(t, ref.W(3,:), 'r', 'LineWidth', 1); hold on;
plot(t, X(:,18), 'k--', 'LineWidth', 1);
xlabel('time(s)'); ylabel('$\boldmath{\Omega}_z$', 'interpreter', 'latex');
% axis([0, 10, -1, 1]);

figure(4)
subplot(311)
plot(t, errors.x(1,:), 'k', 'LineWidth', 1);
% xlabel('time(s)'); 
ylabel('$E_{rx}$', 'interpreter', 'latex');
title('Position tracking errors');
subplot(312)
plot(t, errors.x(2,:), 'k', 'LineWidth', 1); 
% xlabel('time(s)'); 
ylabel('$E_{ry}$', 'interpreter', 'latex');
subplot(313)
plot(t, errors.x(3,:), 'k', 'LineWidth', 1);
xlabel('time(s)'); ylabel('$E_{rz}$', 'interpreter', 'latex');

figure(5)
subplot(311)
plot(t, errors.v(1,:), 'k', 'LineWidth', 1);
% xlabel('time(s)'); 
ylabel('$E_{vx}$', 'interpreter', 'latex');
title('Velocity tracking errors');
subplot(312)
plot(t, errors.v(2,:), 'k', 'LineWidth', 1); 
% xlabel('time(s)'); 
ylabel('$E_{vy}$', 'interpreter', 'latex');
subplot(313)
plot(t, errors.v(3,:), 'k', 'LineWidth', 1);
xlabel('time(s)'); ylabel('$E_{vz}$', 'interpreter', 'latex');

figure(6)
subplot(311)
plot(t, errors.R(1,:), 'k', 'LineWidth', 1);
% xlabel('time(s)'); 
ylabel('$E_{Rx}$', 'interpreter', 'latex');
title('Attitude tracking errors');
subplot(312)
plot(t, errors.R(2,:), 'k', 'LineWidth', 1); 
% xlabel('time(s)'); 
ylabel('$E_{Ry}$', 'interpreter', 'latex');
subplot(313)
plot(t, errors.R(3,:), 'k', 'LineWidth', 1);
xlabel('time(s)'); 
ylabel('$E_{Rz}$', 'interpreter', 'latex');

figure(7)
subplot(311)
plot(t, errors.W(1,:), 'k', 'LineWidth', 1);
% xlabel('time(s)'); 
ylabel('$E_{{\Omega}x}$', 'interpreter', 'latex');
title('Angular velocity tracking errors');
subplot(312)
plot(t, errors.W(2,:), 'k', 'LineWidth', 1); 
% xlabel('time(s)'); 
ylabel('$E_{{\Omega}y}$', 'interpreter', 'latex');
subplot(313)
plot(t, errors.W(3,:), 'k', 'LineWidth', 1);
xlabel('time(s)'); ylabel('$E_{{\Omega}z}$', 'interpreter', 'latex');

figure(8)
plot(t, errors.Phi, 'k', 'LineWidth', 1);
xlabel('time(s)'); ylabel('$\Phi$', 'interpreter', 'latex');
title('Attitude errors');

figure(9)
subplot(411)
plot(t, ctrl.thrust(1,:), 'k', 'LineWidth', 1);
% xlabel('time(s)'); 
ylabel('$f_1$', 'interpreter', 'latex');
title('Thrust');
subplot(412)
plot(t, ctrl.thrust(2,:), 'k', 'LineWidth', 1); 
% xlabel('time(s)'); 
ylabel('$f_2$', 'interpreter', 'latex');
subplot(413)
plot(t, ctrl.thrust(3,:), 'k', 'LineWidth', 1);
% xlabel('time(s)'); 
ylabel('$f_3$', 'interpreter', 'latex');
subplot(414)
plot(t, ctrl.thrust(4,:), 'k', 'LineWidth', 1);
xlabel('time(s)'); ylabel('$f_4$', 'interpreter', 'latex');
