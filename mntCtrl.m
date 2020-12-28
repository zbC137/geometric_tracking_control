function [error, Phi, Wd, M] = mntCtrl(k, param, X, Xd, Bd)

% Split the states
x = X(1:3);
v = X(4:6);
R = reshape(X(7:15), 3, 3);
W = X(16:18);
R_dot = R*hat(W);

xd = Xd(:, 1);
vd = Xd(:, 2);
xd_2dot = Xd(:, 3);
xd_3dot = Xd(:, 4);
xd_4dot = Xd(:, 5);

b1d = Bd(:, 1);
b1d_dot = Bd(:, 2);
b1d_2dot = Bd(:, 3);

% Translation errors
ex = x - xd;
ev = v - vd;

% Desired R
A = -k.x*ex - k.v*ev - param.m*param.g*param.e3 + param.m*xd_2dot;
ea = param.g*param.e3 + dot(A, R*param.e3)*R*param.e3/param.m - xd_2dot;
A_dot = -k.x*ev - k.v*ea + param.m*xd_3dot;

b3d = -A/norm(A);
b3d_dot = -A_dot/norm(A) + A*dot(A, A_dot)/(norm(A)^3);

ej = dot(A_dot, R*param.e3)*R*param.e3/param.m ...
     + dot(A, R_dot*param.e3)*R*param.e3/param.m ...
     + dot(A, R*param.e3)*R_dot*param.e3/param.m- xd_3dot;
A_2dot = -k.x*ea - k.v*ej + param.m*xd_4dot;
[~, ~, b3d_2dot] = derivative(-A, -A_dot, -A_2dot);

B = cross(b3d, b1d);
B_dot = cross(b3d_dot, b1d) + cross(b3d, b1d_dot);
B_2dot = cross(b3d_2dot, b1d) + 2*cross(b3d_dot, b1d_dot)...
         + cross(b3d, b1d_2dot);
[b2d, b2d_dot, b2d_2dot] = derivative(B, B_dot, B_2dot);

b1d = cross(b2d, b3d);
b1d_dot = cross(b2d_dot, b3d) + cross(b2d, b3d_dot);
b1d_2dot = cross(b2d_2dot, b3d) + 2*cross(b2d_dot, b3d_dot) + ...
           cross(b2d, b3d_2dot);

Rd = [b1d, b2d, b3d];
Rd_dot = [b1d_dot, b2d_dot, b3d_dot];
Rd_2dot = [b1d_2dot, b2d_2dot, b3d_2dot];

% Attitude errors
eR = deHat(Rd'*R - R'*Rd)/2;

Wd = deHat(Rd'*Rd_dot);
Wd_dot = deHat(Rd'*Rd_2dot - hat(Wd)^2);
eW = W - R'*Rd*Wd;

% Attitude control for UAV
M = -k.R*eR - k.W*eW + cross(W, param.J*W)...
    - param.J*(hat(W)*R'*Rd*Wd - R'*Rd*Wd_dot);

% Errors
error = [ex, ev, eR, eW];
Phi = trace(eye(3) - Rd'*R)/2;

end

