function dX = Dynamics(t, X, flag, k, param)

% Split the states
x = X(1:3);
v = X(4:6);
R = reshape(X(7:15), 3, 3);
W = X(16:18);

% Reference trajectory
[Xd, Bd] = reference(t, flag);

% Position control
f = posCtrl(k.x, k.v, param.m, param.g, param.e3, R,...
            x, v, Xd);
% Attitude control
[~, ~, ~, M] = mntCtrl(k, param, X, Xd, Bd);

% Equations of motion for UAV
x_dot = v;
v_dot = param.g*param.e3 - f*R*param.e3/param.m;
R_dot = R*hat(W);
W_dot = param.J\(-cross(W, param.J*W) + M);

dX = [x_dot; v_dot; reshape(R_dot, 9, 1); W_dot];

end

