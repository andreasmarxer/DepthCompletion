function lines = getLines(depth_img_woraster, x1, x2, y1, y3, bb_width, debug)
% returns line segments found in bounding box with Canny and Hough Transform
% the frame has a bend if the edge goes out of the bb area because we want
% the frame always to stays inside the bounding box
%
% input:    depth_img_woraster: depth image in uint16 format
%           x1, x2, y1, y3: bounding box coordinates
%           bb_width:   width of frame along detected edges to take
%                       measurements for RANSAC (previously used bounding
%                       box and not detected edges, there the name comes from)
%           debug:      boolean, if true prints detected edges etc.
%
% output:   lines:      struct array with fields: point1, point2, theta, rho

bb_width_half = (bb_width-1)/2;
% extract bounding box area
depth_img_woraster_BB = depth_img_woraster(y1:y3, x1:x2);
% canny edge detection
BW= edge(depth_img_woraster_BB, 'Canny');

%% line detection with hough transform

% 1. make hough transform: rho = x*cos(theta) + y*sin(theta)
[Hough, Theta, Rho] = hough(BW, 'RhoResolution', 1, 'Theta', -90:89);
%[Hough2, Theta2, Rho2] = hough(BW, 'RhoResolution', 1, 'Theta', 45:89);

% 2. visualize peaks of hough transform
num_peaks = 50;
Peaks  = houghpeaks(Hough, num_peaks, 'threshold', ceil(0.2*max(Hough(:)))); % peak
%Peaks2  = houghpeaks(Hough2, num_peaks, 'threshold', ceil(0.5*max(Hough2(:)))); % peak

% 3. find lines
a = 0.0741;
b = 7.7778;
fill_gap_th = round(a*(x2-x1)+b);
min_lenght_th = round(1/3*(x2-x1));
lines = houghlines(BW, Theta, Rho, Peaks, 'FillGap', fill_gap_th, 'MinLength', min_lenght_th);

if debug == true
    figure(10)
    imshow(BW)
    hold on
    figure(11)
    imshow(BW)
    hold on
end