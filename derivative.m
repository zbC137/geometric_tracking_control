function [o, o_dot, o_2dot] = derivative(i, i_dot, i_2dot)

ni = norm(i);
o = i/ni;
o_dot = i_dot/ni - i*dot(i, i_dot)/ni^3;
o_2dot = i_2dot/ni - i_dot*dot(i, i_dot)/ni^3 ...
         -i_dot*dot(i, i_dot)/ni^3 -i*dot(i_dot, i_dot)/ni^3 ...
         -i*dot(i, i_2dot)/ni^3 + 3*i*dot(i, i_dot)^2/ni^5;

end

