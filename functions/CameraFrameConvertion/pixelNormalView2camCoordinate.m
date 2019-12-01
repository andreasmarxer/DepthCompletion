function [points_X_c, points_Y_c, points_Z_c] = pixelNormalView2camCoordinate(P_all)
% convert pixel coordinates of normal (not overhead) view to camera coordinates
% input     u,v are pixel coordinates (origin top left), z is depth [mm]
% output:   x,y,z in camera frame [mm] (origin at cx, cy see in intrinsics)

points_x = single(P_all(:,1));
points_y = single(P_all(:,2));
points_z = single(P_all(:,3));

% initialization
points_Z_c = points_z;
points_X_c = zeros(size(points_z,1),1);
points_Y_c = zeros(size(points_z,1),1);

% camera intrinsic with shifted principal point due to rotated image
load('data/K_normalView.mat'); % loaded as K_normalView
K_inv = inv(K_normalView);

% convert to camera coordinates
    for i=1:size(points_z,1)
        rho = points_Z_c(i);
        XY_c = K_inv(1:2,1:3) .* rho *  [points_x(i); points_y(i); 1];
        points_X_c(i) = -XY_c(1); % needed because of 180° rotation around optical center
        points_Y_c(i) = -XY_c(2); % needed because of 180° rotation around optical center
    end

end
