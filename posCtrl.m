function f = posCtrl(kx, kv, m, g, e3, R, x, v, Xd)

% Split the states
xd = Xd(:, 1);
vd = Xd(:, 2);
xd_2dot = Xd(:, 3);

% errors
ex = x - xd;
ev = v - vd;

% Position control for UAV
A = -kx*ex - kv*ev - m*g*e3 + m*xd_2dot;
f = dot(-A, R*e3);

end

