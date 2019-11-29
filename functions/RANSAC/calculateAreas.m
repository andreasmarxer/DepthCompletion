function [A_P_triangle, A_bb] = calculateAreas(random3, points_x, points_y, x1, x2, y2, y3)
% calculate area of bounding bax and area inside triangle of 3 selected
% points for fitting plane
% inputs:   random3:    index of random selected 3 points of RANSAC
%           points_x:   all x candidate coordinates
%           points_y:   all y candidate coordinates
%           x1, x2, y2, y3: relevant bounding box corners

% output:   areas in [px²]

% two vectors of selected 3 points
v12 = [points_x(random3(2))-points_x(random3(1)); points_y(random3(2))-points_y(random3(1))];
v13 = [points_x(random3(3))-points_x(random3(1)); points_y(random3(3))-points_y(random3(1))];

% length of the two vectors
l12 = sqrt(sum(abs(v12).^2)); %pytagoras: l = sqrt((x2-x1)² + (y2-y1)²)
l13 = sqrt(sum(abs(v13).^2)); %pytagoras: l = sqrt((x3-x1)² + (y3-y1)²)

% angle between the two vectors
alpha = acos(dot(v12,v13)/(l12*l13));

% area of triangle of 3 selected points
A_P_triangle = 1/2 * l12 * l13 * sin(alpha);

% area of current bounding box
A_bb = (x2-x1)*(y3-y2);

end

