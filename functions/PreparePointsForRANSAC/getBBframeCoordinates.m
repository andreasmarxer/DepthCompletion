
function [points_x, points_y] = getBBframeCoordinates(x1,y1, x2,y2, x3,y3, x4,y4, bb_width, bb_width_scale, depth_img)
% return all points inside bb_width frame on the bb detection
% input:    x1,y1, x2,y2, x3,y3, x4,y4: bounding box corners
%           bb_width: width of frame around edge function
%           bb_width_scale: frame width to take points for RANSAC 
%           e.g. [-3,-2,-1,0,1,2,3] for bb_width of 7 and inside=true
%           depth_img: uint16 format

% output:   points_x: x coordinates of all candidate points for RANSAC
%           points_y: y coordinates of all candidate points for RANSAC
%           criterion to be a points is that a) have depth measurement
%           and b) are inside the frame around the bouding box corners


% write bouding box frame coordinates start top left clockwise (1->4)
bb_width_half = (bb_width-1)/2;

% repeate bb_width times per line (first bb_width times 1->2)
points_x_12 = repmat((x1-bb_width_half:x2-bb_width_half-1)', bb_width,1);
points_x_23 = ones(y3-y2,1)*(bb_width_scale+x2);
points_x_34 = repmat((x3+bb_width_half:-1:x4+bb_width_half+1)',bb_width,1);
points_x_41 = ones(y4-y1,1)*(bb_width_scale+x1);
points_x = [points_x_12; points_x_23(:); points_x_34; points_x_41(:)];

points_y_12 = ones(x2-x1,1)*(bb_width_scale+y1);
points_y_23 = repmat((y2-bb_width_half:y3-bb_width_half-1)', bb_width,1);
points_y_34 = ones(x3-x4,1)*(bb_width_scale+y3);
points_y_41 = repmat((y4+bb_width_half:-1:y1+bb_width_half+1)',bb_width,1);
points_y = [points_y_12(:); points_y_23; points_y_34(:); points_y_41];

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
