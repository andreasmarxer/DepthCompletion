function [xs1, ys1, xs2,ys2, xs3,ys3, xs4,ys4] = getIntersectionEdges(x1, x2, y_top, y_bot, x_left, x_right)
% get intersections of edge function
% input:    x1,x2: left and riht bounding box borders
%           y_top, y_bot: functions for edge frame of x defining the y value
%           x_left, x_right: functions for edge frame of y defining the x value
% output:   coordinates of 4 intersection of edge frames inside bounding box
%           important is that these are already in image coordinates [px]

x_vec = x1:x2;

xs1 = find( x_left( y_top( x_vec ) ) == x_vec )+x1-1;
% no intersection on discrete pixel value
if size(xs1,2) == 0 
    xs1 = find( x_left( y_top( x_vec -1 ) ) == x_vec )+x1-1;
% no unambigous intersection (multiple)
elseif size(xs1,2) > 1
    xs1 = xs1(1);
end
ys1 = y_top(xs1);

xs2 = find( x_right( y_top( x_vec ) ) == x_vec )+x1-1;
if size(xs2,2) == 0 % xs4 ot found does not exist
    % intersection may is not in the pixel center
    xs2 = find( x_right( y_top( x_vec -1 ) ) == x_vec )+x1-1;
elseif size(xs2,2) > 1
    xs2 = xs2(1);
end
ys2 = y_top(xs2);

xs3 = find( x_right( y_bot( x_vec ) ) == x_vec )+x1-1;
if size(xs3,2) == 0 % xs4 ot found does not exist
    % intersection may is not in the pixel center
    xs3 = find( x_right( y_bot( x_vec -1 ) ) == x_vec )+x1-1;
elseif size(xs3,2) > 1
    xs3 = xs3(1);
end
ys3 = y_bot(xs3);

xs4 = find( x_left( y_bot( x_vec ) ) == x_vec )+x1-1;
if size(xs4,2) == 0 % xs4 ot found does not exist
    % intersection may is not in the pixel center
    xs4 = find( x_left( y_bot( x_vec-1 ) ) == x_vec )+x1-1;
elseif size(xs4,2) > 1
    xs4 = xs4(1);
end
ys4 = y_bot(xs4);
end