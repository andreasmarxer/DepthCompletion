function [points_X_norm, points_Y_norm] = pixel2normImgCoordinates(x, y)
% convert pixel coordinates (in overhead view / ros bag format) to normalised image coordinates with z=1
%
% input     u,v are pixel coordinates (origin top left), z is depth in mm
% output:   x,y,1 in normalized image coordinates at z=1 [mm] (origin at cx, cy)

% camera intrinsics (saved fram CameraInfo topic in RosBag)
load('data/K.mat'); % loaded as K

% 1. convert to camera coordinates
points_X_norm = zeros(size(x,1),1);
points_Y_norm = zeros(size(x,1),1);

% inverse camera intrinsics
K_inv = inv(K);

    for i=1:size(x,1)
        XY1 = K_inv * [x(i); y(i); 1];
        points_X_norm(i) = XY1(1);
        points_Y_norm(i) = XY1(2);
        %check_if_1 = XY1(3);
    end

end
