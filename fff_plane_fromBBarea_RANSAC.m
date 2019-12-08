clear; clc; close all;
%%=========================================================================
%% parameters
tic;
% 370, 2, 10, 377
% 100 very good with XX params
label =165;  %196

% correcting rastered data
kernel = 5;
kernel_half = (kernel-1)/2;
raster_correction = true; % correct missing depth pixels with kernel

% depth adaptation
depth_method = 1; %1 = only z treshold, 2 = only angle threshold, 3 = combination

depth_procentual_th = 0.30; %procentage of depth measurement available
inlier_th = 20; %mm distance to plane that is considered as inlier
bb_width = 7; %must be odd number, width of frame around bb
inside = true; %if true also bb_width/2 inside the bb area

% intelligent x-y ransac picking criterion
min_max_th = 3; %ratio which the distance max-min of points must have of total bb 
area_th = 1/6; %procentage of area from bb that the triangle of chosen points must cover
% depth picking criterion ransac
depth_dist_th = 1000;

%%=========================================================================

%% fixed dependencies
asl_train_labels = [1,16,39,90,128,178,199,221,264,269,283,289,307,337,350,355,361,363,365,368];
if ismember(label, asl_train_labels)
    error('ERROR: This image was used for training!');
end

path = '/home/andreas/Documents/ASL_window_dataset/';
depth_filename = strcat(path, 'depth_images_mm/' ,'asl_window_', num2str(label), '_depth', '.png');
bb_filename = strcat(path, 'rgb_images_predictions/' ,'asl_window_', num2str(label), '_rgb', '.txt');
rgb_filename = strcat(path, 'rgb_images/' ,'asl_window_', num2str(label), '_rgb', '.jpg');
[~, confidence, left_x_vec, top_y_vec, width_vec, height_vec] = importBoundingBoxes(bb_filename);

%% original depth image
depth_img = imread(depth_filename); % meters

%% rgb image
figure(1)
imshow(rgb_filename)
title(strcat('RGB image - Frame : ', num2str(label)));
plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);
pause(0.2)

%% correcting rastered holes

if raster_correction == true
    depth_img_woraster = depth_img;
    
    for x = kernel_half+1:1:size(depth_img,2)-kernel_half
        for y = kernel_half+1:1:size(depth_img,1)-kernel_half
            depth_img_woraster(y,x) = kernelMedian(3,x,y,depth_img);
        end
    end
    
    depth_img = depth_img_woraster;
end

figure(2)
visualize_depth_png(depth_img)  % millimeters
title(strcat('Depth Image corrected with nonzero median filter (k=', num2str(kernel), ') - Frame : ', num2str(label)));
plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);
pause(0.2)

%% adjust the depth
depth_img_adj = depth_img;
depth_img_corr = uint16(zeros(size(depth_img)));
plane_pts = uint16([]);

for window = 1:size(left_x_vec)
    % get the predicted bounding box (bb) coordinates
    % !!! 1: top left, 2: top right, 3: bottom right, 4: bottom left !!!
    [x1, y1, x2, y2, x3, y3, x4, y4 ] = convertToBBCoords(left_x_vec(window),...
        top_y_vec(window), width_vec(window), height_vec(window), depth_img);
    
    if inside == true
        % inside and outside bb_width_half
        bb_width_scale = -(bb_width-1)/2:1:(bb_width-1)/2;
    
    else
        % only go outside the predicted bounding box for plane fitting
        bb_width_scale = 0:1:bb_width-1;
    end
    
    % get bounding box frame coordinates
    [points_x, points_y] = getBBframeCoordinates(x1,y1, x2,y2, x3,y3, x4,y4, bb_width, bb_width_scale, depth_img);

    % initialize depth with NaNs
    points_z = NaN(size(points_x,1),1);
    n_points_z_total = size(points_z,1);
    
    % assign depth measurment to z coordinate of plane
    for n = 1:size(points_x,1)
        points_z(n) = depth_img(points_y(n),points_x(n));
    end
    
    %% RANSAC
    % if we have more depth measurments than depth_th (%) from hole frame
    % make RANSAC with all this depth measurements, else skip this window
    
    % only consider the points where we have a depth measurement
    if nnz(points_z) > depth_procentual_th * n_points_z_total
        points_x = points_x(points_z>0);
        points_y = points_y(points_z>0);
        points_z = points_z(points_z>0); % points_z must be the last !!!

    else
        disp(strcat('Not enough depth points at window: ', num2str(window)));
        continue % with next window
    end
    
    % RANSAC initialization
    best_inlier = 0; 
    best_param = [0; 0; 0]; 
    best_distance = 0;
    best_std = inf;
    
    ransac_it_skipped = 0;
    while_count = 0;
    
    %% ransac =============================================================
    for ransac_it = 1:500
        %% fit plane
        % initialize not unique!
        random3 = ones(1,3);
        %disp(strcat('RANSAC iteration: ', num2str(ransac_it)));

        % crit = FALSE -> make RANSAC
        % crit = TRUE  -> random pick point again
      
        % initialize
        must_ransac_crit = 1;
        opt_ransac_crit_xy_minmax = 1;
        opt_ransac_crit_z = 1;
        opt_ransac_crit_depth = 1;
        
        while_it = 0;
        % tries n times pick rand number before 1 ransac it is skipped
        n_tries_before_skip_ransac_it = 1000;
        
        %% while loop -----------------------------------------------------

        while opt_ransac_crit_xy_minmax || opt_ransac_crit_area || opt_ransac_crit_depth
            
            % pick random points
            random3 = randi([1 size(points_x,1)],1,3);
            %disp(strcat('while loop it: ', num2str(while_it)));
            
            % Update the criterions
            must_ransac_crit = size(unique(random3),2) ~= 3 || size(unique(points_x(random3)),1) < 2 || size(unique(points_y(random3)),1) < 2;
            opt_ransac_crit_xy_minmax = max(points_x(random3))-min(points_x(random3)) < (x2-x1)/min_max_th || max(points_y(random3))-min(points_y(random3)) < (y4-y1)/min_max_th;
            
            [A_P_triangle, A_bb] = calculateAreas(random3, points_x, points_y, x1, x2, y2, y3); % calculate area inside selected 3 random points
            opt_ransac_crit_area = A_P_triangle < area_th*A_bb;
            
            if depth_method == 2
                opt_ransac_crit_depth = 0; % don't considered
            else
                opt_ransac_crit_depth = max(points_z(random3))-min(points_z(random3)) > depth_dist_th;
            end
    
            
            if while_it > n_tries_before_skip_ransac_it
                disp(strcat('Skipped ransac it #', num2str(ransac_it)));
                ransac_it_skipped = ransac_it_skipped + 1;
                break % out of while loop
            end
            while_it = while_it + 1;
        end
        %% end of while ---------------------------------------------------
        
        while_count = while_count + while_it;
        
        if while_it > n_tries_before_skip_ransac_it
            continue % go to next ransac_it for loop
        end

        % fit plane from 3 points
        P = [points_x(random3), points_y(random3), points_z(random3)];
        param = pts2plane(P); % param = [a; b; d]
        
        %% take all points to calculate distance
        P_all = [points_x, points_y, points_z];
        
        distance = dist2plane(P_all, param);
        inlier = nnz(distance<inlier_th);
        std = sqrt(sum(distance.^2)/size(distance,1));
        
        % Convert to spherical camera coordiantes and calculate angles
        % only used for alternative depth_method
        [theta, phi] = calculateAngles(random3, P_all);
        
        
        if depth_method == 1
            opt_crit_z_angle_fullfilled = 1; %always 1, don't consider this criterion
        else
            opt_crit_z_angle_fullfilled = (abs(theta)<85) && (  ((0<abs(phi)) && (abs(phi)<10)) || ((170<abs(phi)) && (abs(phi)<180)) );
        end
        
        % save new best parameters of ransac
        if inlier > best_inlier && opt_crit_z_angle_fullfilled
            best_inlier = inlier;
            best_param = param;
            best_random3 = random3;
            best_std = std;
            best_theta = theta;
            best_phi = phi;
        end
    
    end
    %% end of ransac ======================================================

    if exist('best_random3', 'var')
        % save all considered points for plane fitting
        plane_pts = [plane_pts; P_all(best_random3,:)];
    end
    
    disp(strcat('Window: ', num2str(window), ' Best inlier ratio: ', num2str(best_inlier/size(points_z,1)), ' Best std: ', num2str(best_std)));
    disp(strcat('Iterations skipped : ', num2str(ransac_it_skipped)));
    disp(strcat('Best theta: ', num2str(best_theta), 'Best phi: ', num2str(best_phi)));

    for x = x1:1:x2
        for y = y1:1:y4
            depth_img_corr(y,x) = mapxy2plane(x, y, best_param);
            % only use it if nothing is in front of corrected window depth
            if depth_img(y,x) == 0 || (depth_img_corr(y,x)<depth_img(y,x) && depth_img_corr(y,x) ~=0)
                depth_img_adj(y,x) = depth_img_corr(y,x);
            end
        end
    end 
    
    % plot points from which plane was fitted in depth image
    if size(plane_pts,1) ~= 0
        % plot them if any plane is fitted
        figure(2)
        hold on
        plot(plane_pts(:,1), plane_pts(:,2),'mo')
    end
    
end

%% visualization

% figure(3)
% visualize_depth_png(depth_img_corr)  % millimeters
% title(strcat('RANSAC plane fitted windows - Frame : ', num2str(label)));
% plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);
% pause(0.2)
% if size(plane_pts,1) ~=0
%     hold on
%     plot(plane_pts(:,1),plane_pts(:,2),'mo')
%     pause(0.2)
% end

toc;

if exist('while_count', 'var')
    disp(strcat('Total while count: ', num2str(while_count)));
end

figure(4)
visualize_depth_png(depth_img_adj)
title(strcat('Corrected depth image (k=', num2str(kernel), ') with RANSAC plane fitted windows - Frame : ', num2str(label)));
plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);
pause(0.5)

% plot points from which plane was fitted in adj depth image
if size(plane_pts,1) ~=0
    hold on
    plot(plane_pts(:,1),plane_pts(:,2),'mo')
    pause(0.5)
end

%%

%adjDepth2PngImage(depth_img_adj, label)

%% functions


function param = pts2plane(P)
% input:    P = [x1, y1, z1; x2, y2, z2; x3, y3, z3]
% output:   param = [a; b; d] for fitting plane z = ax + by + d
% standard plane form is: ax + by + cz + d -> c=-1 in our case !!!

% [x, y, 1] * [a; b; d] = z <-> A*x=B --> x = A\B
param = [P(:,1), P(:,2), ones(size(P,1),1)] \ P(:,3);

end


function z = mapxy2plane(x, y, param)
% z = ax + by + d

z = param(1)*x + param(2)*y + param(3);

end


function distance = dist2plane(P, param)
% input:    P = [x1, y1, z1; x2, y2, z2; x3, y3, z3; ... ; xN, yN, zN]
% output:   distance

a = param(1);
b = param(2);
c = -1;
d = param(3);

N = size(P,1);
distance = zeros(N,1);

    for i = 1:1:N
        distance(i) = abs(a*P(i,1)+b*P(i,2)+c*P(i,3)+d)/sqrt(a^2+b^2+c^2);
        %hold on
        %scatter3(P(i,1), P(i,2), P(i,3), 'g*')
    end

end


function [points_x, points_y] = getBBframeCoordinates(x1,y1, x2,y2, x3,y3, x4,y4, bb_width, bb_width_scale, depth_img)

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


function [A_P_triangle, A_bb] = calculateAreas(random3, points_x, points_y, x1, x2, y2, y3)
            
% two vectors of selected 3 points
v12 = [points_x(random3(2))-points_x(random3(1)); points_y(random3(2))-points_y(random3(1))];
v13 = [points_x(random3(3))-points_x(random3(1)); points_y(random3(3))-points_y(random3(1))];

% length of the two vectors
l12 = sqrt(sum(abs(v12).^2)); %pytagoras: l = sqrt((x2-x1)² + (y2-y1)²)
l13 = sqrt(sum(abs(v13).^2)); %pytagoras: l = sqrt((x3-x1)² + (y3-y1)²)

% angle between the two vectors
alpha = acos(dot(v12,v13)/(l12*l13));

% area of triangle of 3 selected points
A_P_triangle = 1/2 * l12 * l13 * sin(alpha);

% area of current bounding box
A_bb = (x2-x1)*(y3-y2);

end


function [theta, phi] = calculateAngles(random3, P_all)

points_x = P_all(:,1);
points_y = P_all(:,2);
points_z = P_all(:,3);

% 1. convert to camera coordinates
points_Z_c = points_z(random3);
points_X_c = zeros(3,1);
points_Y_c = zeros(3,1);

% camera parameters (saved fram CameraInfo topic in RosBag)
load('K.mat');
K_inv = inv(K);

    for i=1:3
        rho = points_Z_c(i);
        XY_c = K_inv(1:2,1:3) .* rho * [points_x(random3(i)); points_y(random3(i)); 1];
        points_X_c(i) = XY_c(1);
        points_Y_c(i) = XY_c(2);
    end

% 2. calculate plane again in same coordinates
P_c = [points_X_c, points_Y_c, points_Z_c];
param_c = pts2plane(P_c);

% normal vector im [mm] is [a, b, c=-1]
normal_vec_c = [param_c(1); param_c(2); -1];
% normalize and point in positive z direction
normal_vec_c = -1.*normal_vec_c/norm(normal_vec_c);

% 3. spherical coordinates
r = sqrt(sum(normal_vec_c.^2));

theta = rad2deg(acos(normal_vec_c(3)/r));
phi = rad2deg(atan2(normal_vec_c(2), normal_vec_c(1)));

end


function adjDepth2PngImage(depth_img_adj, label)
    
% make changes here ---------------------------------------------------
    type = 'depth_adj';
    filepath =  '/home/andreas/Documents/ASL_window_dataset/depth_images_adj/';
    % ---------------------------------------------------------------------
    
    img = depth_img_adj;

    filename = strcat('asl_window_', num2str(label), '_', num2str(type), '.png');
    imwrite(img, strcat(filepath, filename), 'fmt', 'png');

end


