clear; clc; close all;
%%=========================================================================
%% parameters

%%%%%%%%%%%%%%%%%%%
ploting = true;
save = false;
label = 218;  %52 fails 
% edge detector horizontal tunned with 366
% 126 good for tuning vertical also
%%%%%%%%%%%%%%%%%%%

% correcting rastered data
kernel = 5;
kernel_half = (kernel-1)/2;
raster_correction = true; % correct missing depth pixels with kernel

% depth adaptation
depth_method = 1; %1 = only z treshold, 2 = only angle threshold, 3 = combination

depth_procentual_th = 0.30; % [%] of depth measurements available
inlier_th = 50; %[mm] distance to plane that is considered as inlier
bb_width = 7; %must be odd number, width of frame around bb
inside = true; %if true also bb_width/2 inside the bb area

confidence_th = 95; % [%] above which prediction confidence consider window

% intelligent x-y ransac picking criterion
% min_max_th = 3; %ratio which the distance max-min of points must have of total bb 
area_th = 1/9; %procentage of area from bb that the triangle of chosen points must cover
% depth picking criterion ransac
% depth_dist_th = 500; %[mm]
% theta_th = 85; % [deg] spherical coordinates threshold
% phi_th = 10; % [deg] spherical coordinates threshold

asl_train_labels = [1,16,39,90,128,178,199,221,264,269,283,289,307,337,...
                    350,355,361,363,365,368];           
                
if label == 0
    label_array = 1:1:70;
else
    label_array = label;
end

for label = label_array

    if ismember(label, asl_train_labels)
        disp('Label was used for training, skipped !');
        continue;
    else
        disp(strcat('-Label : ', num2str(label)));
        tic;
        
        %% fix dependencies

        path = '/home/andreas/Documents/ASL_window_dataset/';
        depth_filename = strcat(path, 'depth_images_mm/' ,'asl_window_', num2str(label), '_depth', '.png');
        bb_filename = strcat(path, 'rgb_images_predictions/' ,'asl_window_', num2str(label), '_rgb', '.txt');
        rgb_filename = strcat(path, 'rgb_images/' ,'asl_window_', num2str(label), '_rgb', '.jpg');
        [class, confidence, left_x_vec, top_y_vec, width_vec, height_vec] = importBoundingBoxes(bb_filename);
        
        %% only consider windows with a confidence higher than the th
        windows_conf = confidence>confidence_th;
        left_x_vec = left_x_vec(windows_conf);
        top_y_vec = top_y_vec(windows_conf);
        width_vec = width_vec(windows_conf);
        height_vec = height_vec(windows_conf);
        
        disp(strcat(num2str(nnz(windows_conf)), {' from '} , num2str(size(windows_conf,1)), ' windows with enough confidence considered'));
        
        %% original depth image
        depth_img = imread(depth_filename); % meters

%         if ploting == true
%             figure(1)
%             visualize_depth_png(depth_img)  % millimeters
%             title(strcat('Original Depth Image - Frame:', num2str(label)));
%             plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);
%             pause(0.2) % otherwise sometimes this isn't plotted
%         end

        %% rgb image
        
%         if ploting == true
%             figure(2)
%             imshow(rgb_filename)
%             title(strcat('RGB image - Frame : ', num2str(label)));
%             plot_bb(left_x_vec,top_y_vec,width_vec,height_vec,depth_img);
%             pause(0.2) % otherwise sometimes this isn't plotted
%         end

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

        if ploting == true
            figure(2)
            visualize_depth_png(depth_img)  % millimeters
            title(strcat('Depth Image corrected with nonzero median filter (k=', num2str(kernel), ') - Frame : ', num2str(label)));
            plot_bb(left_x_vec,top_y_vec,width_vec,height_vec,depth_img);
            pause(0.2) % otherwise sometimes this isn't plotted
        end

        %% adjust the depth
        depth_img_adj = depth_img;
        depth_img_corr = uint16(zeros(size(depth_img)));
        plane_pts = uint16([]);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for window = 1:size(left_x_vec)
            % get the predicted bounding box (bb) coordinates
            % !!! 1: top left, 2: top right, 3: btm right, 4: btm left !!!
            [x1, y1, x2, y2, x3, y3, x4, y4 ] = ...
                convertToBBCoords(left_x_vec(window), top_y_vec(window), ...
                width_vec(window), height_vec(window), depth_img);

            if inside == true
                % inside and outside bb_width_half
                bb_width_scale = -(bb_width-1)/2:1:(bb_width-1)/2;
            else
                % only go outside the predicted bb for plane fitting
                bb_width_scale = 0:1:bb_width-1;
            end

            %% 1. get bounding box frame coordinates
            [points_x, points_y] = getBBframeCoordinates(x1,y1, x2,y2, ...
                x3,y3, x4,y4, bb_width, bb_width_scale, depth_img);

            % initialize depth with NaNs
            points_z = NaN(size(points_x,1),1);
            n_points_z_total = size(points_z,1);

            % assign depth measurment to z coordinate of plane
            for n = 1:size(points_x,1)
                points_z(n) = depth_img(points_y(n),points_x(n));
            end

            % only if more depth measurments than depth_th (%) in bb frame
            if nnz(points_z) > depth_procentual_th * n_points_z_total
                % only consider the points where we have a depth measurement
                points_x = points_x(points_z>0);
                points_y = points_y(points_z>0);
                points_z = points_z(points_z>0); 
                % points_z must be the last otherwise this doesn't work!!!
            else
                disp(strcat('Not enough depth points at window: ', num2str(window)));
                continue % with next window (for loop iteration)
            end

            %% 2. convert to camera frame coordinates (from px coordinates)
            
            P_all = [points_x, points_y, points_z];
            [points_X_c, points_Y_c, points_Z_c] = pixel2camCoordinate(P_all);
                            
            %% 3. make RANSAC in camera frame
            
            % RANSAC initialization
            best_inlier = 0; 
            best_param = [0; 0; 0]; 
            best_distance = 0;
            best_std = inf;

            ransac_it_skipped = 0;
            while_count = 0;
            
            %% ransac =====================================================
            for ransac_it = 1:500
                %% fit plane
                % initialize not unique!
                random3 = ones(1,3);
                %disp(strcat('RANSAC iteration: ', num2str(ransac_it)));

                % crit = FALSE -> make RANSAC
                % crit = TRUE  -> random pick point again

                % initialize
                must_ransac_crit = 1;
                opt_ransac_crit_area = 1;
                
                while_it = 1;
                n_tries_before_skip_ransac_it = 1000;
        
                %% while loop ---------------------------------------------
                while must_ransac_crit || opt_ransac_crit_area
                    
                    % pick three random points
                    random3 = randi([1 size(points_X_c,1)],1,3);
                    %disp(strcat('while loop it: ', num2str(while_it)));

                    % Update the criterions
                    must_ransac_crit = size(unique(random3),2) ~= 3 || ...
                        size(unique(points_X_c(random3)),1) < 2 || ...
                        size(unique(points_Y_c(random3)),1) < 2;
                    
                    [A_P_triangle, A_bb] = ...
                        calculateAreas(random3, points_x, points_y, ...
                        x1, x2, y2, y3); % calculate area inside random3
                    opt_ransac_crit_area = A_P_triangle < area_th*A_bb;


                    if while_it > n_tries_before_skip_ransac_it
                        %disp(strcat('Skipped ransac it #', num2str(ransac_it)));
                        ransac_it_skipped = ransac_it_skipped + 1;
                        break % out of while loop
                    end
                    while_it = while_it + 1;
                    
                end
                %% end of while -------------------------------------------
                
                while_count = while_count + while_it;
                
                if while_it > n_tries_before_skip_ransac_it
                    continue % go to next ransac_it for loop
                end
        
                % fit plane from 3 points
                P = [points_X_c(random3), points_Y_c(random3), points_Z_c(random3)];
                param = pts2plane(P); % param = [a; b; d] in camera frame

                %% take all points to calculate distance
                P_all_c = [points_X_c, points_Y_c, points_Z_c];
                distance = dist2plane(P_all_c, param);
                inlier = nnz(distance<inlier_th);
                std = sqrt(sum(distance.^2)/size(distance,1));
            
                % save new best parameters of ransac
                if inlier > best_inlier
                    best_inlier = inlier;
                    best_param = param;
                    best_random3 = random3;
                    best_std = std;
                end

            end
            %% end of ransac ==============================================
            
            if exist('best_random3', 'var')
                % save all considered points for plane fitting
                plane_pts = [plane_pts; P_all(best_random3,:)];
            end

            disp(strcat('Window: ', num2str(window), ' Best inlier ratio: ', num2str(best_inlier/size(points_z,1)), ' Best std: ', num2str(best_std)));
            disp(strcat('Iterations skipped : ', num2str(ransac_it_skipped)));
            if depth_method ~=1
                disp(strcat('Best theta: ', num2str(best_theta), 'Best phi: ', num2str(best_phi)));
            end
            
            
            %% edge detection
            depth_img_woraster_BB = depth_img_woraster(y2:y3, x1:x2);
            BW= edge(depth_img_woraster_BB, 'Canny');
            
            %% line detection with hough transform
            
            % 1. make hough transform: rho = x*cos(theta) + y*sin(theta)
            [Hough, Theta, Rho] = hough(BW, 'RhoResolution', 1, 'Theta', -90:89);
            
            % 2. visualize peaks of hough transform
            num_peaks = 20;
            Peaks  = houghpeaks(Hough, num_peaks, 'threshold', ceil(0.1*max(Hough(:)))); % peak
            
            % 3. visualize the fitted lines
            lines = houghlines(BW, Theta, Rho, Peaks, 'FillGap', 30, 'MinLength', 30);
            figure(3)
            imshow(BW)
            hold on

            % 4. calculte top and bot start and end point in bb coordinates
            [xy_bb_top, xy_bb_bot] = calculate_topBot_lines(lines, y1, y3);
            
            % 5. convert line to function of x coordinate
            for x = x1:x2
                if xy_bb_top ~= 0
                    y_top(x) = bbPoints2EdgeBorder(x, xy_bb_top, y1, y3, x1);
                else
                    y_top(x) = y1;
                end
                if xy_bb_bot ~= 0
                    y_bot(x) = bbPoints2EdgeBorder(x, xy_bb_bot, y1, y3, x1);
                else
                    y_bot(x) = y3;
                end
            end
            % round to integer values for use with pixels
            y_top= round(y_top);
            y_bot= round(y_bot);            

            % adjust depth image for each pixel inside actual window
            for x = x1:1:x2
                for y = y_top(x):1:y_bot(x)
                    % convert x,y [px] to normalized image coordinates x,y [m] (z=1)
                    [X_norm, Y_norm] = pixel2normImgCoordinates(x, y);
                    
                    % calculate new Z value
                    % n*[x_n; y_n; 1] = [X_c; Y_c; Z_c] 
                    % where X_c, Y_c, Z_c is element of the plane 
                    new_value = -best_param(3) / (best_param(1)*X_norm + best_param(2)*Y_norm -1);
                    
                    % use always if no depth actually available
                    if depth_img(y,x) == 0 && depth_img_corr(y,x) == 0
                        depth_img_corr(y,x) = new_value;
                        depth_img_adj(y,x) = new_value;
                    % only use it if nothing is in front of adjusted depth
                    % and the new depth is not zero
                    elseif (new_value<depth_img(y,x) && ...
                            new_value<depth_img_adj(y,x))
                        depth_img_corr(y,x) = new_value;
                        depth_img_adj(y,x) = new_value;
                    else
                        continue;
                    end
                end
            end 

            % plot points from which plane was fitted in depth image
            if size(plane_pts,1) ~= 0 && ploting == true
                % plot them if any plane is fitted
                figure(2)
                hold on
                plot(plane_pts(:,1), plane_pts(:,2),'mo')
            end
            
        end
        % end of window for loop
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %% visualization

        if ploting == true
            
%             figure(3)
%             visualize_depth_png(depth_img_corr)  % millimeters
%             title(strcat('RANSAC plane fitted windows - Frame : ', num2str(label)));
%             plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);
%             pause(0.2) % otherwise sometimes this isn't plotted
%             if size(plane_pts,1) ~=0
%                 hold on
%                 plot(plane_pts(:,1),plane_pts(:,2),'mo')
%                 pause(0.2) % otherwise sometimes this isn't plotted
%             end

            figure(4)
            visualize_depth_png(depth_img_adj)
            title(strcat('Corrected depth image (k=', num2str(kernel), ') with RANSAC plane fitted windows - Frame : ', num2str(label)));
            plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);
            pause(0.5) % otherwise sometimes this isn't plotted
            if size(plane_pts,1) ~=0
                hold on
                plot(plane_pts(:,1),plane_pts(:,2),'mo')
                pause(1) % otherwise sometimes this isn't plotted
            end
            
        end

        %%
        if save == true
            adjDepth2PngImage(depth_img_adj, label)
            pause(0.2); % otherwise some images don't get saved
        end
        
        toc;
        
    end % end of else
end % end of label for loop


%% functions


function param = pts2plane(P)
% input:    P = [x1, y1, z1; x2, y2, z2; x3, y3, z3]
% output:   param = [a; b; d] for fitting plane z = ax + by + d
% standard plane form is: ax + by + cz + d -> c=-1 in our case !!!

% [x, y, 1] * [a; b; d] = z <-> A*x=B --> x = A\B
% produces the solution using Gaussian elimination, without explicitly forming the inverse
param = [P(:,1), P(:,2), ones(size(P,1),1)] \ P(:,3);
end


function [points_X_norm, points_Y_norm] = pixel2normImgCoordinates(x, y)
% Input     u,v are pixel coordinates (origin top left), z is depth in mm
% Output:   x,y,1 in normalized image coordinates at z=1

% 1. convert to camera coordinates
points_X_norm = zeros(size(x,1),1);
points_Y_norm = zeros(size(x,1),1);

% camera parameters (saved fram CameraInfo topic in RosBag)
load('K.mat');
K_inv = inv(K);

    for i=1:size(x,1)
        XY1 = K_inv * [x(i); y(i); 1];
        points_X_norm(i) = XY1(1);
        points_Y_norm(i) = XY1(2);
        %check_if_1 = XY1(3);
    end

end


function [points_X_c, points_Y_c, points_Z_c] = pixel2camCoordinate(P_all)
% Input     u,v are pixel coordinates (origin top left), z is depth [mm]
% Output:   x,y,z in camera frame [mm]

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

end


function z = mapxy2plane(x, y, param)
% input:    x,y coordinates in pixel from origin at top left
% output:   mapped value of depth according to fitted plane
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

%% hough transform

function [xy_bb_top, xy_bb_bot] = calculate_topBot_lines(lines, y1, y3)
% initialization
max_len_top = 0;
max_len_bot = 0;
line_of_interest = nan(length(lines),1);
xy_bb_top = 0;
xy_bb_bot = 0;


y_bb_half = floor((y3-y1)/2); % no + y1 due to the fact that this is in BB coords

for k = 1:length(lines)

    % only lines that are 45 or horizontal, not interessted in vertical
    line_of_interest(k) = 45 < abs(lines(k).theta) && abs(lines(k).theta) < 90;
    if line_of_interest(k) == true

        %line_top
        if lines(k).point1(2) < y_bb_half && lines(k).point2(2) < y_bb_half
            disp('top line')
            len = norm(lines(k).point1 - lines(k).point2);
            if (len > max_len_top)
            max_len_top = len;
                % endpoint of longest top line
                xy_bb_top = [lines(k).point1; lines(k).point2];
            end
        %line_bottom
        elseif lines(k).point1(2) > y_bb_half && lines(k).point2(2) > y_bb_half
            disp('bottom line')
            len = norm(lines(k).point1 - lines(k).point2);
            if (len > max_len_bot)
                max_len_bot = len;
                % endpoint of longest bot line
                xy_bb_bot = [lines(k).point1; lines(k).point2];
            end
        end
    end

end

end


function param = pts2line(xy)
% input:    xy = [x1, y1; x2, y2]
% output:   param = [a; b] for fitting line y = ax + b

x1 = xy(1,1);
y1 = xy(1,2);
x2 = xy(2,1);
y2 = xy(2,2);

% [y1; y2] = [x1, 1; x2, 1] * [a; b] <-> A*x=B --> x = A\B
% produces the solution using Gaussian elimination, without explicitly forming the inverse
param = [x1, 1; x2, 1]\[y1; y2];
end


function y = bbPoints2EdgeBorder(x, xy_bb, y1, y3, x1)
param = pts2line(xy_bb); 
a = param(1);
b= param(2);
% We fitted the function in bounding box coordinates y_bb = a*x_bb + b
% But now are interested in y = f(x) in pixel coordinates
% Insert for y_bb and x_bb leads to:    y-y1 = a*(x-x1) + b;

y = a*x + b -a*x1 + y1;

if y < y1 % line is above bounding box
    y = y1;
elseif y > y3 % line is below bounding box
    y = y3;
end

end


%% save image

function adjDepth2PngImage(depth_img_adj, label)
    
% make changes here ---------------------------------------------------
    type = 'depth_adj';
    filepath =  '/home/andreas/Documents/ASL_window_dataset/depth_images_adj/temp/';
    % ---------------------------------------------------------------------
    
    img = depth_img_adj;

    filename = strcat('asl_window_', num2str(label), '_', num2str(type), '.png');
    imwrite(img, strcat(filepath, filename), 'fmt', 'png');

end