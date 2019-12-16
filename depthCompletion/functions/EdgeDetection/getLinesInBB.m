function lines = getLinesInBB(depth_img_woraster, x1, x2, y1, y3)
% returns line segments found in bounding box with Canny and Hough Transform
% the frame has a bend if the edge goes out of the bb area because we want
% the frame always to stays inside the bounding box
%
% input:    depth_img_woraster: depth image in uint16 format
%           x1, x2, y1, y3: bounding box coordinates
%
% output:   lines:      struct array with fields: point1, point2, theta, rho

% extract bounding box area
depth_img_woraster_BB = depth_img_woraster(y1:y3, x1:x2);
% canny edge detection
BW= edge(depth_img_woraster_BB, 'Canny');

%% line detection with hough transform

% 1. make hough transform: rho = x*cos(theta) + y*sin(theta)
[Hough, Theta, Rho] = hough(BW, 'RhoResolution', 1, 'Theta', -90:89);

% 2. take peaks of hough transform
num_peaks = 50;
Peaks  = houghpeaks(Hough, num_peaks, 'threshold', ceil(0.2*max(Hough(:)))); % peak

% 3. set parameters for finding lines
% parameters a,b, such that the fill_gap_th is:
% 10 for a bounding box width of 100px
%  6 for a bounding box width of 10px
a = 0.04;
b = 6;
fill_gap_th = round(a*(x2-x1)+b); 
min_lenght_th = round(1/4*(x2-x1));

% 4. find lines
lines = houghlines(BW, Theta, Rho, Peaks, 'FillGap', fill_gap_th, 'MinLength', min_lenght_th);

end