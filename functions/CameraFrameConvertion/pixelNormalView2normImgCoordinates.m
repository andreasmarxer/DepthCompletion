function [points_X_norm, points_Y_norm] = pixelNormalView2normImgCoordinates(x, y, K_normalView)
% convert pixel coordinates of normal (not overhead) view to normalised image coordinates with z=1
%
% input     u,v are pixel coordinates (origin top left), z is depth in mm
% output:   x,y,1 in normalized image coordinates at z=1 [mm] (origin at cx, cy)

% 1. convert to camera coordinates
points_X_norm = zeros(size(x,1),1);
points_Y_norm = zeros(size(x,1),1);

% inverse camera intrinsics
K_inv = inv(K_normalView);

    for i=1:size(x,1)
        XY1 = K_inv * [x(i); y(i); 1];
        points_X_norm(i) = -XY1(1);
        points_Y_norm(i) = -XY1(2);
        %check_if 1 = XY1(3);
    end

end