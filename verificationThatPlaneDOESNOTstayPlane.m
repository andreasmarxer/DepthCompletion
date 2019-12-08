% generate plane in pixel coordinates
P_px = P_all(best_random3,:);

param_pixel = pts2plane(P_px);

%% transform plane to camera coordinates

%param_pixel = [1;1;1];

% get plane coordiantes
plane_x = repmat((x1:1:x2)', y3-y2+1,1);
points_y_= ones(x2-x1+1,1)*(y2:1:y3);
plane_y = points_y_(:);

plane_z = param_pixel(1).*plane_x + param_pixel(2).*plane_y + param_pixel(3);

figure(11)
scatter3(plane_x, plane_y, plane_z, '.')
xlabel('u')
ylabel('v')
zlabel('z')

%%
P_plane = [plane_x, plane_y, plane_z];


% transform to cam coordinates
[points_X_c, points_Y_c, points_Z_c] = pixel2camCoordinate(P_plane);

P_plane_c = [points_X_c, points_Y_c, points_Z_c];

%%

figure(12)
scatter3(points_X_c, points_Y_c, points_Z_c, '.')
xlabel('X_c')
ylabel('Y_c')
zlabel('Z_c')

hold on
scatter3(points_X_c(best_random3), points_Y_c(best_random3), points_Z_c(best_random3),'ro')


% now for each pixel get the new Z value

%points_Z_c_new = param_cam(1).*points_X_c + param_cam(2).*points_Y_c + param_cam(3);


% for u and v loop
% calculate corresponding X_c and Y_c
% get the value from the fitted plane
%%



%% functions

function param = pts2plane(P)
% input:    P = [x1, y1, z1; x2, y2, z2; x3, y3, z3]
% output:   param = [a; b; d] for fitting plane z = ax + by + d
% standard plane form is: ax + by + cz + d -> c=-1 in our case !!!

% [x, y, 1] * [a; b; d] = z <-> A*x=B --> x = A\B
% produces the solution using Gaussian elimination, without explicitly forming the inverse
param = [P(:,1), P(:,2), ones(size(P,1),1)] \ P(:,3);
end


function [points_X_c, points_Y_c, points_Z_c] = pixel2camCoordinate(P_all)
% x,y are pixel coordinates with origin at top left
% z is depth in mm

points_x = P_all(:,1);
points_y = P_all(:,2);
points_z = P_all(:,3);

% 1. convert to camera coordinates
points_Z_c = points_z;
points_X_c = zeros(size(points_z,1),1);
points_Y_c = zeros(size(points_z,1),1);

% camera parameters (saved fram CameraInfo topic in RosBag)
load('K.mat');
K_inv = inv(K);

    for i=1:size(points_z,1)
        rho = points_Z_c(i);
        XY_c = K_inv(1:2,1:3) .* rho * [points_x(i); points_y(i); 1];
        points_X_c(i) = XY_c(1);
        points_Y_c(i) = XY_c(2);
    end


% load('K.mat');
% K_inv = inv(K);
% 
% Z_c = z;
% X_c = Z_c*(x-x0)/alpha_x;
% Y_c = Z_c*(y-y0)/alpha_y;

end