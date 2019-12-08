function param = pts2plane(P)
% fit a plane from more than 3 points using gaussian elimination
%
% input:    P = [x1, y1, z1; x2, y2, z2; x3, y3, z3]
% output:   param = [a; b; d] for fitting plane z = ax + by + d
% standard plane form is: ax + by + cz + d -> c=-1 in our case !!!

% [x, y, 1] * [a; b; d] = z <-> A*x=B --> x = A\B
% produces the solution using Gaussian elimination, without explicitly forming the inverse
param = [P(:,1), P(:,2), ones(size(P,1),1)] \ P(:,3);
end
