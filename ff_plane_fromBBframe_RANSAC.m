clear; clc; close all;

label = 2;
kernel = 5;
kernel_half = (kernel-1)/2;

depth_th = 0.2;

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

figure(1)
imshow(rgb_filename)
title(strcat('RGB image - Frame : ', num2str(label)));
plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);
pause(0.2)

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

figure(2)
visualize_depth_png(depth_img)  % millimeters
title(strcat('Depth Image corrected with nonzero median filter (k=', num2str(kernel), ') - Frame : ', num2str(label)));
plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);
pause(0.2)

%% adjust the depth
depth_img_adj = depth_img;
depth_img_corr = uint16(zeros(size(depth_img)));

for window = 1:size(left_x_vec)

    [x1, y1, x2, y2, x3, y3, x4, y4 ] = convertToBBCoords(left_x_vec(window),...
        top_y_vec(window), width_vec(window), height_vec(window), depth_img);
    
    % write bouding box frame coordinates start top left clockwise (1->4)
    points_x = [(x1:x2-1)'; ones(y3-y2,1)*x2; (x3:-1:x4+1)'; ones(y4-y1,1)*x1];
    points_y = [ones(x2-x1,1)*y1; (y2:y3-1)'; ones(x3-x4,1)*y3; (y4:-1:y1+1)'];
    
    % initialize depth with NaNs
    points_z = NaN(size(points_x,1),1);
    % assign depth measurment to z coordinate of plane
    for n = 1:size(points_x,1)
        points_z(n) = depth_img(points_y(n),points_x(n));
        %points_z_test(n) = kernelMedian(kernel, points_x(n), points_y(n), depth_img);
    end
    
    %% RANSAC
    % if we have more depth measurments than depth_th (%) from hole frame
    % make RANSAC with all this depth measurements, else skip this window
    
    % only consider the points where we have a depth measurement
    if nnz(points_z) > depth_th * (2*(x2-x1)+2*(y3-y2))
        points_x = points_x(points_z>0);
        points_y = points_y(points_z>0);
        points_z = points_z(points_z>0); % points_z must be the last !!!

    else
        continue
    end
    
    % real RANSAC start
    best_inlier = 0; 
    best_param = [0; 0; 0]; 
    best_distance = 0;
    best_std = inf;
    
    threshold = 10; %mm
    
    for ransac_it = 1:700
        %% fit plane
        % initialize not unique!
        random3 = ones(1,3);
        %disp(strcat('RANSAC iteration: ', num2str(ransac_it)));

        % generate 3 random numbers from scope while 
        % 1) not unique 3 numbers  OR
        % 2) not for all 3 a depth measurement OR
        % 3) 3 same x or y coordinates
        while size(unique(random3),2) ~= 3 || size(unique(points_x(random3)),1) < 2 || size(unique(points_y(random3)),1) < 2
            random3 = randi([1 size(points_x,1)],1,3);
            %disp('while loop');
        end

        P = [points_x(random3), points_y(random3), points_z(random3)];
        param = pts2plane(P);

        %% take all points to calculate distance
        P_all = [points_x, points_y, points_z];
        
        distance = dist2plane(P_all, param);
        inlier = nnz(distance<threshold);
        std = sqrt(sum(distance.^2)/size(distance,1));
        
        if inlier > best_inlier && std < best_std
            best_inlier = inlier;
            best_param = param;
            best_distance = distance;
            best_std = std;
        end
    
    end
    
    disp(strcat('Window: ', num2str(window), ' Best inlier ratio: ', num2str(best_inlier/size(points_z,1)), 'Best std: ', num2str(best_std)));
    
    for x = x1:1:x2
        for y = y1:1:y4
            depth_img_corr(y,x) = mapxy2plane(x, y, best_param);
            % only use it if nothing is in front of corrected window depth
            if depth_img(y,x) == 0 || (depth_img_corr(y,x)<depth_img(y,x) && depth_img_corr(y,x) ~=0)
                depth_img_adj(y,x) = depth_img_corr(y,x);
            end
        end
    end
    
end
%%

%%
figure(3)
visualize_depth_png(depth_img_corr)  % millimeters
title(strcat('RANSAC plane fitted windows - Frame : ', num2str(label)));
plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);
pause(0.2)

figure(4)
visualize_depth_png(depth_img_adj)
title(strcat('Corrected depth image (k=', num2str(kernel), ') with RANSAC plane fitted windows - Frame : ', num2str(label)));
plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);
pause(1)

%% functions

% Assuming that the 3d data P is N-by-3 numeric array,  P = [x, y, z]
% Output a, b and d for fitting plane,                  z = ax + by + d
function param = pts2plane(P)
% input:    P = [x1, y1, z1; x2, y2, z2; x3, y3, z3]
% output:   param = [a; b; d]

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

