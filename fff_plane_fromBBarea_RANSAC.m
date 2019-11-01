clear; clc; close all;
%%=========================================================================
%% parameters
tic;
% 370, 2, 10, 377
% 100 very good with XX params
label = 1;

% correcting rastered data
kernel = 5;
kernel_half = (kernel-1)/2;

% depth adaptation
depth_procentual_th = 0.20; %procentage of depth measurements available
depth_dist_th = 250;
inlier_th = 10; %mm distance to plane that is considered as inlier
bb_width = 7; %must be odd number, width of frame around bb
inside = true; %if true also bb_width/2 inside the bb area

%%=========================================================================

%% fix dependencies
asl_train_labels = [1,16,39,90,128,178,199,221,264,269,283,289,307,337,350,355,361,363,365,368];
if ismember(label, asl_train_labels)
    error('ERROR: This image was used for training!');
end

path = '/home/andreas/Documents/ASL_window_dataset/';
depth_filename = strcat(path, 'depth_images_mm/' ,'asl_window_', num2str(label), '_depth', '.png');
bb_filename = strcat(path, 'rgb_images_predictions/' ,'asl_window_', num2str(label), '_rgb', '.txt');
rgb_filename = strcat(path, 'rgb_images/' ,'asl_window_', num2str(label), '_rgb', '.jpg');
[class, confidence, left_x_vec, top_y_vec, width_vec, height_vec] = importBoundingBoxes(bb_filename);

%% original depth image
depth_img = imread(depth_filename); % meters

% figure(1)
% visualize_depth_png(depth_img)  % millimeters
% title(strcat('Original Depth Image - Frame:', num2str(label)));
% plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);
% pause(0.2)

%% rgb image
% figure(1)
% imshow(rgb_filename)
% title(strcat('RGB image - Frame : ', num2str(label)));
% plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);
% pause(0.2)

%% correcting rastered holes
raster_correction = true;

if raster_correction == true
    depth_img_woraster = depth_img;
    
    for x = kernel_half+1:1:size(depth_img,2)-kernel_half
        for y = kernel_half+1:1:size(depth_img,1)-kernel_half
            depth_img_woraster(y,x) = kernelMedian(3,x,y,depth_img);
        end
    end
    
    depth_img = depth_img_woraster;
end

% figure(2)
% visualize_depth_png(depth_img)  % millimeters
% title(strcat('Depth Image corrected with nonzero median filter (k=', num2str(kernel), ') - Frame : ', num2str(label)));
% plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);
% pause(0.2)

%% adjust the depth
depth_img_adj = depth_img;
depth_img_corr = uint16(zeros(size(depth_img)));
plane_pts = uint16([]);

for window = 1:size(left_x_vec)
    % get the predicted bounding box (bb) coordinates
    % 1: top left, 2: top right, 3: bottom right, 4: bottom left
    [x1, y1, x2, y2, x3, y3, x4, y4 ] = convertToBBCoords(left_x_vec(window),...
        top_y_vec(window), width_vec(window), height_vec(window), depth_img);
    
    if inside == true
        % inside and outside bb_width_half
        bb_width_scale = [-(bb_width-1)/2:1:(bb_width-1)/2];
    
    else
        % only go outside the predicted bounding box for plane fitting
        bb_width_scale = [0:1:bb_width-1];
    end
    
    % write bouding box frame coordinates start top left clockwise (1->4)
    % repeate bb_width times per line (first bb_width times 1->2)
    points_x_12 = repmat((x1:x2-1)',bb_width,1);
    points_x_23 = ones(y3-y2,1)*(bb_width_scale+x2);
    points_x_34 = repmat((x3:-1:x4+1)',bb_width,1);
    points_x_41 = ones(y4-y1,1)*(bb_width_scale+x1);
    points_x = [points_x_12; points_x_23(:); points_x_34; points_x_41(:)];
    
    points_y_12 = ones(x2-x1,1)*(bb_width_scale+y1);
    points_y_23 = repmat((y2:y3-1)',bb_width,1);
    points_y_34 = ones(x3-x4,1)*(bb_width_scale+y3);
    points_y_41 = repmat((y4:-1:y1+1)',bb_width,1);
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
        continue
    end
    
    % RANSAC initialization
    best_inlier = 0; 
    best_param = [0; 0; 0]; 
    best_distance = 0;
    best_std = inf;
    
    ransac_it_skipped = 0;
    
    for ransac_it = 1:500
        %% fit plane
        % initialize not unique!
        random3 = ones(1,3);
        %disp(strcat('RANSAC iteration: ', num2str(ransac_it)));

        % generate 3 random numbers from scope while 
        % 1) not unique 3 numbers  OR
        % 2) 3 same x or y coordinates
        % -> must_crit is FALSE if RANSAC can be made with this points
        must_ransac_crit = size(unique(random3),2) ~= 3 ||...
            size(unique(points_x(random3)),1) < 2 ||...
            size(unique(points_y(random3)),1) < 2;
        % to improve quality of fitted planes also consider:
        % min and max of x and y must differ at least of half kernel!
        % z measurements of picked points don't differ more than
        % depth_diff_th
        % 3 points are spreaded 
        % opt is FALSE if RANSAC is probable to have intelligent picks
        opt_ransac_crit_xy_minmax = (max(points_x(random3))-min(points_x(random3)) < (x2-x1)/2) ||...
            (max(points_y(random3))-min(points_y(random3)) < (y4-y1)/2);
        opt_ransac_crit_z = max(points_z(random3))-min(points_z(random3)) > depth_dist_th;
        opt_ransac_crit_x_1to2 = abs(points_x(random3(1))-points_x(random3(2))) < (x2-x1)/2;
        opt_ransac_crit_y_1to3 = abs(points_y(random3(1))-points_y(random3(3))) < (y3-y2)/2;
        opt_ransac_crit_x_2to3 = abs(points_x(random3(2))-points_x(random3(3))) < (x2-x1)/2;
        opt_ransac_crit_y_2to3 = abs(points_y(random3(2))-points_y(random3(3))) < (y3-y2)/2;
        
        while_it = 1;
        %while must_ransac_crit || opt_ransac_crit_xy_minmax || opt_ransac_crit_z
        while must_ransac_crit || opt_ransac_crit_z || opt_ransac_crit_x_1to2 || opt_ransac_crit_y_1to3 || opt_ransac_crit_x_2to3 || opt_ransac_crit_y_2to3
            random3 = randi([1 size(points_x,1)],1,3);
            %disp(strcat('while loop it: ', num2str(while_it)));
            
            % Update the criterions
            must_ransac_crit = size(unique(random3),2) ~= 3 || size(unique(points_x(random3)),1) < 2 || size(unique(points_y(random3)),1) < 2;
            opt_ransac_crit_xy_minmax = max(points_x(random3))-min(points_x(random3)) < (x2-x1)/2 || max(points_y(random3))-min(points_y(random3)) < (y4-y1)/2;
            opt_ransac_crit_z = max(points_z(random3))-min(points_z(random3)) > 250;
            opt_ransac_crit_x_1to2 = abs(points_x(random3(1))-points_x(random3(2))) < (x2-x1)/3;
            opt_ransac_crit_y_1to3 = abs(points_y(random3(1))-points_y(random3(3))) < (y3-y2)/3;
            opt_ransac_crit_x_2to3 = abs(points_x(random3(2))-points_x(random3(3))) < (x2-x1)/3;
            opt_ransac_crit_y_2to3 = abs(points_y(random3(2))-points_y(random3(3))) < (y3-y2)/3;
            
            while_it = while_it + 1;
            if while_it > 1000
                ransac_it_skipped = ransac_it_skipped + 1;
                break % out of while loop
            end
        end

        if while_it > 1000
            continue % go to next ransac_it for loop
        end
        
        % fit plane from 3 points
        P = [points_x(random3), points_y(random3), points_z(random3)];
        param = pts2plane(P);
        
        %% take all points to calculate distance
        P_all = [points_x, points_y, points_z];
        
        distance = dist2plane(P_all, param);
        inlier = nnz(distance<inlier_th);
        std = sqrt(sum(distance.^2)/size(distance,1));
        
        if inlier > best_inlier %&& std < best_std
            best_inlier = inlier;
            best_param = param;
            best_random3 = random3;
            best_std = std;
        end
    
    end
    
    disp(strcat('Window: ', num2str(window), ' Best inlier ratio: ', num2str(best_inlier/size(points_z,1)), ' Best std: ', num2str(best_std)));
    disp(strcat('Iterations skipped : ', num2str(ransac_it_skipped)));
    
    %if best_std<inlier_th
        for x = x1:1:x2
            for y = y1:1:y4
                depth_img_corr(y,x) = mapxy2plane(x, y, best_param);
                % only use it if nothing is in front of corrected window depth
                if depth_img(y,x) == 0 || (depth_img_corr(y,x)<depth_img(y,x) && depth_img_corr(y,x) ~=0)
                    depth_img_adj(y,x) = depth_img_corr(y,x);
                end
            end
        end 
    %end
    
    % save all considered points for plane fitting
    plane_pts = [plane_pts; P_all(best_random3,:)];
%     if size(plane_pts,1) ~= 0
%         % plot them if any plane is fitted
%         figure(2)
%         hold on
%         plot(plane_pts(:,1),plane_pts(:,2),'mo')
%     end
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

figure(4)
visualize_depth_png(depth_img_adj)
title(strcat('Corrected depth image (k=', num2str(kernel), ') with RANSAC plane fitted windows - Frame : ', num2str(label)));
plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);
pause(0.5)
if size(plane_pts,1) ~=0
    hold on
    plot(plane_pts(:,1),plane_pts(:,2),'mo')
    pause(0.2)
end

%%

adjDepth2PngImage(depth_img_adj, label)

%% functions


function param = pts2plane(P)
% input:    P = [x1, y1, z1; x2, y2, z2; x3, y3, z3]
% output:   param = [a; b; d] for fitting plane z = ax + by + d

% [x, y, 1] * [a; b; c] = z <-> A*x=B --> x = A\B
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


function adjDepth2PngImage(depth_img_adj, label)
    
% make changes here ---------------------------------------------------
    type = 'depth_adj';
    filepath =  '/home/andreas/Documents/ASL_window_dataset/depth_images_adj/';
    % ---------------------------------------------------------------------
    
    img = depth_img_adj;

    filename = strcat('asl_window_', num2str(label), '_', num2str(type), '.png');
    imwrite(img, strcat(filepath, filename), 'fmt', 'png');

end