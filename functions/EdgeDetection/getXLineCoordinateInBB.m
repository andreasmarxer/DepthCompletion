function x = getXLineCoordinateInBB(y, xy_bb, x1, x2, y1)
% calculate y coordinate from line equation as y=f(x)
% input:    y: coordinate to calculate x=f(y)
%           xy = [x1, y1; x2, y2] points defining line
%           x1, x3: left and right borders from bounding box
%           y1: top border
% output:   x=f(y) 
% old function name was bbPoints2XEdgeBorder !!!

param = pts2line(xy_bb); 
a = param(1);
b= param(2);
% We fitted the function in bounding box coordinates y_bb = a*x_bb + b
% But now are interested in x = f(y) in pixel coordinates
% Insert for y_bb and x_bb leads to:    y-y1 = a*(x-x1) + b;

x = (1/a)*y + 1/a*(-y1-b+a*x1);

if x < x1 % line is on left side out of bounding box
    x = x1;
elseif x > x2 % line is on right side out of bounding box
    x = x2;
end

end