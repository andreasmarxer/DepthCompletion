function [y_top, y_bot, x_left, x_right] = line2functionInBB(x1, x2, y1, y3, xy_bb_top, xy_bb_bot, xy_bb_left,xy_bb_right,  bb_width, inside)
% return functions describing the frame along the selected edges
% the frame has a bend if the edge goes out of the bb area because we want
% the frame always to stays inside the bounding box
%
% input:    x1, x2, y1, y3: bounding box coordinates
%           bb_width:   width of frame along detected edges to take
%                       measurements for RANSAC (previously used bounding
%                       box and not detected edges, there the name comes from)
%           inside:     boolean,  if true also bb_width/2 inside the edge frame
%                                 if false only bb_width outside the edge frame
%
% output:   functions y=f(x) and x=f(y) defining the frame on which the
%           window frame is presumed in !!! image coordinates !!!

if inside == true
    % we go half of the bb_width inside and half outside the edges
    bb_width_half = (bb_width-1)/2;
else
    % we go the hole bb_with outside the edges
    bb_width_half = bb_width;
end

if x1-bb_width_half > 1 && x2+bb_width_half < 640
    x_ = x1-bb_width_half:x2+bb_width_half;
elseif x1-bb_width_half > 1
    x_ = x1-bb_width_half:640;
elseif x2+bb_width_half < 640
    x_ = 1:x2+bb_width_half;
else
    x_ = x1:x2;
end
if y1-bb_width_half > 1 && y3+bb_width_half < 480
    y_ = y1-bb_width_half:y3+bb_width_half;
else
    y_ = y1:y3;
end
    
for x = x_
    if xy_bb_top ~= 0
        % calculate y coordinate from line function as y=f(x)
        y_top(x) = getYLineCoordinateInBB(x, xy_bb_top, y1, y3, x1);
    else
        % take bounding box limit
        y_top(x) = y1;
    end
    if xy_bb_bot ~= 0
        % calculate y coordinate from line function as y=f(x)
        y_bot(x) = getYLineCoordinateInBB(x, xy_bb_bot, y1, y3, x1);
    else
        % take bounding box limit
        y_bot(x) = y3;
    end
end

for y = y_
    if size(xy_bb_left,1) ~= 1 && ~(xy_bb_left(1,1)==xy_bb_left(2,1)) %2nd criterion is due to singularity of function y=f(x) can't map x=2
        % calculate x coordinate from line function as x=f(y)
        x_left(y) = getXLineCoordinateInBB(y, xy_bb_left, x1, x2, y1);
    else
        % take bounding box limit
        x_left(y) = x1;
    end
    if size(xy_bb_right,1) ~= 1 && ~(xy_bb_right(1,1)==xy_bb_right(2,1))
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