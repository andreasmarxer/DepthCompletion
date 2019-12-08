function param = pts2line(xy)
% get line parameter from 2 points
% input:    xy = [x1, y1; x2, y2]
% output:   param = [a; b] for fitting line y = ax + b

x1 = xy(1,1);
y1 = xy(1,2);
x2 = xy(2,1);
y2 = xy(2,2);

% [y1; y2] = [x1, 1; x2, 1] * [a; b] <-> A*x=B --> x = A\B
% produces the solution using Gaussian elimination, without explicitly forming the inverse
param = [x1, 1; x2, 1]\[y1; y2];

end