function [points_x, points_y] = getEdgeframeCoordinates(xs1,ys1, xs2,ys2, xs3,ys3, xs4,ys4, y_top, y_bot, x_left, x_right, bb_width, bb_width_scale, depth_img)
% return all points inside bb_width frame around detected edge functions
% input:    xs1,ys1, xs2,ys2, xs3,ys3, xs4,ys4: intersection of edge function
%           y_top, y_bot, x_left, x_right:
%           bb_width: width of frame around edge function
%           bb_width_scale: frame width to take points for RANSAC 
%           e.g. [-3,-2,-1,0,1,2,3] for bb_width of 7 and inside=true
%           depth_img: uint16 format

% output:   points_x: x coordinates of all candidate points for RANSAC
%           points_y: y coordinates of all candidate points for RANSAC
%           criterion to be a points is that a) have depth measurement
%           and b) are inside the frame around the detected edge function

% write bouding box frame coordinates start top left clockwise (1->4)
bb_width_half = (bb_width-1)/2;

% write bouding box frame coordinates start top left clockwise (1->4)
bb_width_half = (bb_width-1)/2;

% care, this now are non square boxes anymore !!!
if xs1-bb_width>1 && xs4-bb_width>1 && ys1-bb_width>1 && ys2-bb_width>1 ...
    && xs3+bb_width < size(depth_img,2) && ys4+bb_width < size(depth_img,1)
    % can make nice corner overlaps
    points_x_12 = repmat((xs1-bb_width_half:xs2-bb_width_half-1)', bb_width, 1);
    points_y_23 = repmat((ys2-bb_width_half:ys3-bb_width_half-1)', bb_width, 1);
    points_x_34 = repmat((xs3+bb_width_half:-1:xs4+bb_width_half+1)', bb_width, 1);
    points_y_41 = repmat((ys4+bb_width_half:-1:ys1+bb_width_half+1)', bb_width, 1);

    points_y_12 = ones(xs2-xs1,1) * bb_width_scale + repmat((y_top(xs1-bb_width_half:xs2-bb_width_half-1))',1,bb_width);
    points_y_12 = points_y_12(:);
    points_x_23 = ones(ys3-ys2,1) * bb_width_scale + repmat((x_right(ys2-bb_width_half:ys3-bb_width_half-1))',1,bb_width);
    points_x_23 = points_x_23(:);
    points_y_34 = ones(xs3-xs4,1) * bb_width_scale + repmat((y_bot(xs3+bb_width_half:-1:xs4+bb_width_half+1))',1,bb_width);
    points_y_34 = points_y_34(:);
    points_x_41 = ones(ys4-ys1,1) * bb_width_scale + repmat((x_left(ys4+bb_width_half:-1:ys1+bb_width_half+1))',1,bb_width);
    points_x_41 = points_x_41(:); 
else
    % start at xs1 instead of xs1-bb_width_half
    % --> corner has empty part
    points_x_12 = repmat((xs1:xs2-1)', bb_width, 1);
    points_x_34 = repmat((xs3:-1:xs4+1)', bb_width, 1);
    points_y_12 = ones(xs2-xs1,1) * bb_width_scale + repmat((y_top(xs1:xs2-1))',1,bb_width);
    points_y_12 = points_y_12(:);
    points_y_34 = ones(xs3-xs4,1) * bb_width_scale + repmat((y_bot(xs3:-1:xs4+1))',1,bb_width);
    points_y_34 = points_y_34(:);
    % rest is equal
    points_y_23 = repmat((ys2:ys3-1)', bb_width, 1);
    points_y_41 = repmat((ys4:-1:ys1+1)', bb_width, 1);

    points_x_23 = ones(ys3-ys2,1) * bb_width_scale + repmat((x_right(ys2:ys3-1))',1,bb_width);
    points_x_23 = points_x_23(:);
    points_x_41 = ones(ys4-ys1,1) * bb_width_scale + repmat((x_left(ys4:-1:ys1+1))',1,bb_width);
    points_x_41 = points_x_41(:);
end

%% Debugging
%     figure(2)
%     hold on; plot(points_x_12(1:end),points_y_12(1:end),'x');
%     hold on; plot(points_x_23(1:end),points_y_23(1:end),'x');
%     hold on; plot(points_x_34(1:end),points_y_34(1:end),'x');
%     hold on; plot(points_x_41(1:end),points_y_41(1:end),'x');

%% summarize output 
points_x = [points_x_12; points_x_23; points_x_34; points_x_41];
points_y = [points_y_12; points_y_23; points_y_34; points_y_41];

%% check if still inside image
% right border
points_x = points_x(points_x<size(depth_img,2));
points_y = points_y(points_x<size(depth_img,2));
% bottom border
points_y = points_y(points_y<size(depth_img,1));
points_x = points_x(points_y<size(depth_img,1));
% left border
points_x = points_x(points_x>0);
points_y = points_y(points_x>0);
% top border
points_y = points_y(points_y>0);
points_x = points_x(points_y>0);

return;
    
end
