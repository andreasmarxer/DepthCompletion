function [points_X_c, points_Y_c, points_Z_c] = pixel2camCoordinate(P_all)
% convert pixel coordinates (in overhead view / ros bag format) to camera coordinates
% input     u,v are pixel coordinates (origin top left), z is depth [mm]
% output:   x,y,z in camera frame [mm] (origin at cx, cy see in intrinsics)

points_x = P_all(:,1);
points_y = P_all(:,2);
points_z = P_all(:,3);

% 1. convert to camera coordinates
points_Z_c = points_z;
points_X_c = zeros(size(points_z,1),1);
points_Y_c = zeros(size(points_z,1),1);

% camera intrinsic (saved fram CameraInfo topic in RosBag)
load('data/K.mat');
K_inv = inv(K);

    for i=1:size(points_z,1)
        rho = points_Z_c(i);
        XY_c = K_inv(1:2,1:3) .* rho * [points_x(i); points_y(i); 1];
        points_X_c(i) = XY_c(1);
        points_Y_c(i) = XY_c(2);
    end

end
