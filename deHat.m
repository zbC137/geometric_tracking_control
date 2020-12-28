function [vector] = deHat(matrix)
% map from a skew symmetry matrix to a vector
vector = [matrix(3,2); matrix(1,3); matrix(2,1)];
end

