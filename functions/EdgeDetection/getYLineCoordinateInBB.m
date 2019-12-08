function y = getYLineCoordinateInBB(x, xy_bb, y1, y3, x1)
% calculate y coordinate from line equation as y=f(x)
% input:    x: coordinate to calculate y=f(x)
%           xy = [x1, y1; x2, y2] points defining line
%           y1, y3: top and bottom borders from bounding box
%           x1: left border
% output:   y=f(x) 
% old function name was bbPoints2YEdgeBorder !!!

param = pts2line(xy_bb); 
a = param(1);
b= param(2);
% We fitted the function in bounding box coordinates y_bb = a*x_bb + b
% But now are interested in y = f(x) in pixel coordinates
% Insert for y_bb and x_bb leads to:    y-y1 = a*(x-x1) + b;


y = a*x + b -a*x1 + y1;

if y < y1 % line is above bounding box
    y = y1;
elseif y > y3 % line is below bounding box
    y = y3;
end

end