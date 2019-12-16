function [y_top, y_bot, x_left, x_right] = line2functionInBB(x1, x2, y1, y3, xy_bb_top, xy_bb_bot, xy_bb_left,xy_bb_right)
% return functions describing the frame along the selected edges
% the frame has a bend if the edge goes out of the bb area because we want
% the frame always to stays inside the bounding box
%
% input:    x1, x2, y1, y3: bounding box coordinates
%
% output:   functions y=f(x) and x=f(y) defining the frame on which the
%           window frame is presumed in !!! image coordinates !!!


if x1 > 1 && x2 < 640
    x_ = x1:x2;
elseif x1 > 1
    x_ = x1:640;
elseif x2 < 640
    x_ = 1:x2;
else
    x_ = x1:x2;
end
if y1 > 1 && y3 < 480
    y_ = y1:y3;
else
    y_ = y1:y3;
end
    
for x = x_
    if all(all(~isnan(xy_bb_top)))
        % calculate y coordinate from line function as y=f(x)
        y_top(x) = getYLineCoordinateInBB(x, xy_bb_top, y1, y3, x1);
    else
        % take bounding box limit
        y_top(x) = y1;
    end
    if all(all(~isnan(xy_bb_bot)))
        % calculate y coordinate from line function as y=f(x)
        y_bot(x) = getYLineCoordinateInBB(x, xy_bb_bot, y1, y3, x1);
    else
        % take bounding box limit
        y_bot(x) = y3;
    end
end

for y = y_
    if all(all(~isnan(xy_bb_left))) && ~(xy_bb_left(1,1)==xy_bb_left(2,1)) %2nd criterion is due to singularity of function y=f(x) can't map x=2
        % calculate x coordinate from line function as x=f(y)
        x_left(y) = getXLineCoordinateInBB(y, xy_bb_left, x1, x2, y1);
    else
        % take bounding box limit
        x_left(y) = x1;
    end
    if all(all(~isnan(xy_bb_right))) && ~(xy_bb_right(1,1)==xy_bb_right(2,1))
        % calculate x coordinate from line function as x=f(y)
        x_right(y) = getXLineCoordinateInBB(y, xy_bb_right, x1, x2, y1);
    else
        % take bounding box limit
        x_right(y) = x2;
    end
end

% round to integer values for use with pixels
y_top= round(y_top);
y_bot= round(y_bot);
x_left = round(x_left);
x_right= round(x_right);

end