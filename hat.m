function [matrix] = hat(vector)
% map from a vector to a skew symmetry matrix
matrix = [         0, -vector(3),  vector(2)
           vector(3),          0, -vector(1)
          -vector(2),  vector(1),          0];
end

