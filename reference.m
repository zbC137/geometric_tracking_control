function [X, B] = reference(t, flag)
% Grenerate reference trajectory
switch(flag)
    case 1
        x = [0.4*t; 0.4*sin(pi*t); -0.6*cos(pi*t)];
        v = [0.4; 0.4*pi*cos(pi*t); 0.6*pi*sin(pi*t)];
        a = [0; -0.4*pi^2*sin(pi*t); 0.6*pi^2*cos(pi*t)];
        j = [0; -0.4*pi^3*cos(pi*t); -0.6*pi^3*sin(pi*t)];
        s = [0; 0.4*pi^4*sin(pi*t); -0.6*pi^4*cos(pi*t)];
        
        b = [cos(pi*t); sin(pi*t); 0];
        b_dot = [-pi*sin(pi*t); pi*cos(pi*t); 0];
        b_2dot = [-pi^2*cos(pi*t); -pi^2*sin(pi*t); 0];
        
    case 2
        x = zeros(3, 1);
        v = zeros(3, 1);
        a = zeros(3, 1);
        j = zeros(3, 1);
        s = zeros(3, 1);
        
        b = [1; 0; 0];
        b_dot = zeros(3, 1);
        b_2dot = zeros(3, 1);
        
end

X = [x, v, a, j, s];
B = [b, b_dot, b_2dot];
end

