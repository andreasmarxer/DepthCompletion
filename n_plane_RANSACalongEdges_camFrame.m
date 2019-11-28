clear; clc; close all;
%%=========================================================================
%% parameters

%%%%%%%%%%%%%%%%%%%
ploting = true;
rotate_back = true; % rotate the processed image before saving png
% images need to have corrected upright  view due to prediction bouding box format
debug = false;
save = false;
label = 100:200;
% edge detector horizontal tunned with 366
% working well % % % % % % % % 
% 126, 131: one open door
% 198:      one side door    
% 303, 304: one dark door
% 360:      two doors
% 366, 367: one open door
% % % % % % % % % % % % % % % %
% FAILS: 52, 298, 301, 302
%%%%%%%%%%%%%%%%%%%

depth_procentual_th = 0.25; % [%] of depth measurements available
inlier_th = 50; % [mm] distance to plane that is considered as inlier
bb_width = 7; % must be odd number, width of frame around bb
inside = true; % if true also bb_width/2 inside the bb area

confidence_th = 95; % [%] above which prediction confidence consider window

area_th = 1/9; % procentage of area from bb that the triangle of chosen points must cover

% labels which were used for training network, not adapted and considered !
asl_train_labels = [1,16,39,90,128,178,199,221,264,269,283,289,307,337,...
                    350,355,361,363,365,368];           
                
if label == 0
    label_array = 1:1:488; 
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
        depth_filename_median = strcat(path, 'depth_images_median5/','asl_window_', num2str(label), '_depth_median5', '.png');
        bb_filename = strcat(path, 'rgb_images_predictions/' ,'asl_window_', num2str(label), '_rgb', '.txt');
        rgb_filename = strcat(path, 'rgb_images/' ,'asl_window_', num2str(label), '_rgb', '.jpg');
        [class, confidence, left_x_vec, top_y_vec, width_vec, height_vec] = importBoundingBoxes(bb_filename);
        
        %% only consider windows with a confidence higher than the threshold
        windows_conf = confidence>confidence_th;
        left_x_vec = left_x_vec(windows_conf);
        top_y_vec = top_y_vec(windows_conf);
        width_vec = width_vec(windows_conf);
        height_vec = height_vec(windows_conf);
        
        disp(strcat(num2str(nnz(windows_conf)), {' from '} , num2str(size(windows_conf,1)), ' windows with enough confidence considered'));
        
        %% median filtered (kernel 5x5) depth image
        depth_img = imread(depth_filename_median); % meters

        if ploting == true
            figure(1)
            visualize_depth_png(depth_img)  % millimeters
            title(strcat('Depth Image corrected with nonzero median filter (k=5) - Frame : ', num2str(label)));
            % plots predicted bounding box corner points
            plot_bb(left_x_vec,top_y_vec,width_vec,height_vec,depth_img);
            pause(0.2) % otherwise sometimes this isn't plotted
        end
        
        %% rgb image
        if ploting == true
            figure(2)
            imshow(rgb_filename)
            % plots predicted bounding box corner points
            plot_bb(left_x_vec,top_y_vec,width_vec,height_vec,depth_img);
            pause(0.2) % otherwise sometimes this isn't plotted
        end
        %% adjust the depth image
        % initialization
        depth_img_adj = depth_img;  % finally contains depth measurment and fitted planes
        depth_img_corr = uint16(zeros(size(depth_img))); % only contains fitted planes
        plane_pts = uint16([]); % points chosen to fit plane for all predictions

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % for every prediction in this image
        for window = 1:size(left_x_vec)
            % get the predicted bounding box coordinates
            % !!! 1: top left, 2: top right, 3: btm right, 4: btm left !!!
            [x1, y1, x2, y2, x3, y3, x4, y4 ] = ...
                convertToBBCoords(left_x_vec(window), top_y_vec(window), ...
                width_vec(window), height_vec(window), depth_img);
       
            
            %% edge detection
            % get edge functions which define frame around window
            [y_top, y_bot, x_left, x_right] = holeEdgeShit(depth_img, ...
                x1, x2, y1, y3, bb_width, debug);
            
            % get intersections of edge function
            [xs1, ys1, xs2,ys2, xs3,ys3, xs4,ys4] = getIntersectionEdges(...
                x1, x2, y_top, y_bot, x_left, x_right);
            
            % define frame width to take points for RANSAC
            if inside == true
                % inside and outside bb_width_half
                bb_width_scale = -(bb_width-1)/2:1:(bb_width-1)/2;
            else
                % only go outside the predicted bb for plane fitting
                bb_width_scale = 0:1:bb_width-1;
            end   
            
            %% 1. get all coordinates of points in bounding box edge frame
            [points_x, points_y] = getEdgeframeCoordinates(xs1,ys1,...
                xs2,ys2, xs3,ys3, xs4,ys4, y_top, y_bot, x_left, x_right,...
                bb_width, bb_width_scale, depth_img);
            
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
            
            %% rgb image
            % plot edges in rgb image
            if ploting == true
                figure(2)
                hold on
                plot(x1:x2, y_top(x1:x2))
                hold on
                plot(x_right(y2:y3), y2:y3)
                hold on
                plot(x1:x2, y_bot(x1:x2))
                hold on
                plot(x_left(y2:y3), y2:y3)
                hold on
                % plot all coordinates in bounding box edge frame
%                 plot(points_x, points_y,'.');
%                 hold on
                % plot intersection of edges in rgb
                plot([xs1;xs2;xs3;xs4],[ys1;ys2;ys3;ys4], 'y+');
                hold on
            end

            %% 2. convert from image coordinates [px] to camera frame coordinates [mm]

            % use the intrinsics to convert to camera coordinates
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
            
            %% RANSAC ======================================================
            for ransac_it = 1:500
                % initialize not unique!
                random3 = ones(1,3);

                % initialize (crit=1/true means we do pick again)
                must_ransac_crit = 1;
                opt_ransac_crit_area = 1;
                
                while_it = 1;
                n_tries_before_skip_ransac_it = 1000;
        
                % while loop ---------------------------------------------
                while must_ransac_crit || opt_ransac_crit_area
                    
                    % pick three random points
                    random3 = randi([1 size(points_X_c,1)],1,3);

                    % update the criterions that we have 3 different points
                    must_ransac_crit = size(unique(random3),2) ~= 3 || ...
                        size(unique(points_X_c(random3)),1) < 2 || ...
                        size(unique(points_Y_c(random3)),1) < 2;
                    
                    % update the criterion that we have a minimal area between the points
                    [A_P_triangle, A_bb] = ...
                        calculateAreas(random3, points_x, points_y, ...
                        x1, x2, y2, y3); 
                    opt_ransac_crit_area = A_P_triangle < area_th*A_bb;
                    
                    % if did not find 3 points that match the criterions in 1000 iterations
                    if while_it > n_tries_before_skip_ransac_it
                        ransac_it_skipped = ransac_it_skipped + 1;
                        break % out of while loop
                    end
                    while_it = while_it + 1;
                    
                end
                % end of while -------------------------------------------
                
                while_count = while_count + while_it;
                
                if while_it > n_tries_before_skip_ransac_it
                    continue % go to next ransac_it for loop
                end
        
                % fit plane from 3 points
                P = [points_X_c(random3), points_Y_c(random3), points_Z_c(random3)];
                param = pts2plane(P); % param = [a; b; d] in camera frame

                % take all points to calculate distance
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

            % camera intrinsics (saved fram CameraInfo topic in RosBag)
            load('data/K.mat');

            % adjust depth image for each pixel inside bounding box
            for x = x1:1:x2
                for y = y1:1:y3
                    % only adjust if we are also inside the chose window edges
                    if y_top(x)<=y && y_bot(x)>=y && x_left(y)<=x && x_right(y)>=x
                        
                        % convert x,y [px] to normalized image coordinates x,y [m] (z=1m)
                        [X_norm, Y_norm] = pixel2normImgCoordinates(x, y, K);

                        % calculate new Z value
                        % n*[x_n; y_n; 1] = [X_c; Y_c; Z_c] 
                        % where X_c, Y_c, Z_c is element of the plane 
                        new_value = -best_param(3) / (best_param(1)*X_norm + best_param(2)*Y_norm -1);

                        % use the new value for every pixel no depth is available
                        if depth_img(y,x) == 0 && depth_img_corr(y,x) == 0
                            depth_img_corr(y,x) = new_value; 
                            depth_img_adj(y,x) = new_value;
                        % only use the new value for pixels where nothing is in front of adjusted depth
                        % and the new depth is not zero
                        elseif (new_value<depth_img(y,x) && ...
                                new_value<depth_img_adj(y,x))
                            depth_img_corr(y,x) = new_value;
                            depth_img_adj(y,x) = new_value;
                        else
                            continue;
                        end
                    
                    else
                        continue % with next for loop
                    end % of if
                end % of for y
            end % of for x
            
        end
        % end of window for loop
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %% visualization
        % plot all points from which plane was fitted in depth image if any plane was fitted
        if size(plane_pts,1) ~= 0 && ploting == true
            figure(2)
            hold on
            plot(plane_pts(:,1), plane_pts(:,2),'mo')
        end
            
        if ploting == true
            
            figure(3)
            pause(0.2)
            visualize_depth_png(depth_img_adj)
            pause(0.2)
            title(strcat('Corrected depth image (k=5) with RANSAC plane fitted windows - Frame : ', num2str(label)));
            % plots predicted bounding box corner points
            plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);
            pause(0.5) % otherwise sometimes this isn't plotted
            if size(plane_pts,1) ~=0
                hold on
                plot(plane_pts(:,1),plane_pts(:,2),'mo')
                pause(0.5) % otherwise sometimes this isn't plotted
                figure(1)
                hold on
                plot(plane_pts(:,1),plane_pts(:,2),'mo')
                pause(0.5) % otherwise sometimes this isn't plotted
            end
            
        end

        %% save adjusted depth image as png
        if save == true
            
            adjDepth2PngImage(depth_img_adj, label, rotate_back)
            pause(0.2); % otherwise some images don't get saved
        end
        
        toc;
        
    end % end of else
end % end of label for loop


%% functions


function param = pts2plane(P)
% fit a plane from more than 3 points using gaussian elimination
%
% input:    P = [x1, y1, z1; x2, y2, z2; x3, y3, z3]
% output:   param = [a; b; d] for fitting plane z = ax + by + d
% standard plane form is: ax + by + cz + d -> c=-1 in our case !!!

% [x, y, 1] * [a; b; d] = z <-> A*x=B --> x = A\B
% produces the solution using Gaussian elimination, without explicitly forming the inverse
param = [P(:,1), P(:,2), ones(size(P,1),1)] \ P(:,3);
end


function [points_X_norm, points_Y_norm] = pixel2normImgCoordinates(x, y, K)
% convert pixel coordinates to normalised image coordinates with z=1
%
% input     u,v are pixel coordinates (origin top left), z is depth in mm
% output:   x,y,1 in normalized image coordinates at z=1 [mm] (origin at cx, cy)

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


function [points_X_c, points_Y_c, points_Z_c] = pixelNormalView2camCoordinate(P_all)
% convert pixel coordinates of normal (not overhead) view to camera coordinates
% input     u,v are pixel coordinates (origin top left), z is depth [mm]
% output:   x,y,z in camera frame [mm] (origin at cx, cy see in intrinsics)

points_x = P_all(:,1);
points_y = P_all(:,2);
points_z = P_all(:,3);

% initialization
points_Z_c = points_z;
points_X_c = zeros(size(points_z,1),1);
points_Y_c = zeros(size(points_z,1),1);

% camera intrinsic (saved fram CameraInfo topic in RosBag)
load('data/K_normalView.mat');
K_inv = inv(K_normalView);

% convert to camera coordinates
    for i=1:size(points_z,1)
        rho = points_Z_c(i);
        XY_c = K_inv(1:2,1:3) .* rho *  [points_x(i); points_y(i); 1];
        points_X_c(i) = -XY_c(1); % needed because of 180° rotation around optical center
        points_Y_c(i) = -XY_c(2); % needed because of 180° rotation around optical center
    end

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


function [points_x, points_y] = getEdgeframeCoordinates(xs1,ys1, xs2,ys2, xs3,ys3, xs4,ys4, y_top, y_bot, x_left, x_right, bb_width, bb_width_scale, depth_img)

    % write bouding box frame coordinates start top left clockwise (1->4)
    bb_width_half = (bb_width-1)/2;
    
    % write bouding box frame coordinates start top left clockwise (1->4)
    bb_width_half = (bb_width-1)/2;
    
    % care, this now are non square boxes anymore !!!
    if xs1-bb_width>1 && xs4-bb_width>1 && ys1-bb_width>1 && ys2-bb_width>1 ...
        && xs3+bb_width < size(depth_img,2) && ys4+bb_width < size(depth_img,1)
        % can make nice corner overlaps
        points_x_12 = repmat((xs1-bb_width_half:xs2-bb_width_half-1)', bb_width, 1);
        points_y_23 = repmat((ys2-bb_width_half:ys3-bb_width_half-1)', bb_width, 1);
        points_x_34 = repmat((xs3+bb_width_half:-1:xs4+bb_width_half+1)', bb_width, 1);
        points_y_41 = repmat((ys4+bb_width_half:-1:ys1+bb_width_half+1)', bb_width, 1);

        points_y_12 = ones(xs2-xs1,1) * bb_width_scale + repmat((y_top(xs1-bb_width_half:xs2-bb_width_half-1))',1,bb_width);
        points_y_12 = points_y_12(:);
        points_x_23 = ones(ys3-ys2,1) * bb_width_scale + repmat((x_right(ys2-bb_width_half:ys3-bb_width_half-1))',1,bb_width);
        points_x_23 = points_x_23(:);
        points_y_34 = ones(xs3-xs4,1) * bb_width_scale + repmat((y_bot(xs3+bb_width_half:-1:xs4+bb_width_half+1))',1,bb_width);
        points_y_34 = points_y_34(:);
        points_x_41 = ones(ys4-ys1,1) * bb_width_scale + repmat((x_left(ys4+bb_width_half:-1:ys1+bb_width_half+1))',1,bb_width);
        points_x_41 = points_x_41(:); 
    else
        % start at xs1 instead of xs1-bb_width_half
        % --> corner has empty part
        points_x_12 = repmat((xs1:xs2-1)', bb_width, 1);
        points_x_34 = repmat((xs3:-1:xs4+1)', bb_width, 1);
        points_y_12 = ones(xs2-xs1,1) * bb_width_scale + repmat((y_top(xs1:xs2-1))',1,bb_width);
        points_y_12 = points_y_12(:);
        points_y_34 = ones(xs3-xs4,1) * bb_width_scale + repmat((y_bot(xs3:-1:xs4+1))',1,bb_width);
        points_y_34 = points_y_34(:);
        % rest is equal
        points_y_23 = repmat((ys2:ys3-1)', bb_width, 1);
        points_y_41 = repmat((ys4:-1:ys1+1)', bb_width, 1);

        points_x_23 = ones(ys3-ys2,1) * bb_width_scale + repmat((x_right(ys2:ys3-1))',1,bb_width);
        points_x_23 = points_x_23(:);
        points_x_41 = ones(ys4-ys1,1) * bb_width_scale + repmat((x_left(ys4:-1:ys1+1))',1,bb_width);
        points_x_41 = points_x_41(:);
    end
    
    %% Debugging
%     figure(2)
%     hold on; plot(points_x_12(1:end),points_y_12(1:end),'x');
%     hold on; plot(points_x_23(1:end),points_y_23(1:end),'x');
%     hold on; plot(points_x_34(1:end),points_y_34(1:end),'x');
%     hold on; plot(points_x_41(1:end),points_y_41(1:end),'x');
            
    %% summarize output 
    points_x = [points_x_12; points_x_23; points_x_34; points_x_41];
    points_y = [points_y_12; points_y_23; points_y_34; points_y_41];
    
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


function param = pts2line(xy)
% get line parameter from 2 points
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


function y = bbPoints2YEdgeBorder(x, xy_bb, y1, y3, x1)
% calculate y coordinate from line equation as y=f(x)
% input:    x: coordinate to calculate y=f(x)
%           xy = [x1, y1; x2, y2] points defining line
%           y1, y3: top and bottom borders from bounding box
%           x1: left border
% output:   y=f(x) 

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


function x = bbPoints2XEdgeBorder(y, xy_bb, x1, x2, y1)
% calculate y coordinate from line equation as y=f(x)
% input:    y: coordinate to calculate x=f(y)
%           xy = [x1, y1; x2, y2] points defining line
%           x1, x3: left and right borders from bounding box
%           y1: top border
% output:   x=f(y) 

param = pts2line(xy_bb); 
a = param(1);
b= param(2);
% We fitted the function in bounding box coordinates y_bb = a*x_bb + b
% But now are interested in x = f(y) in pixel coordinates
% Insert for y_bb and x_bb leads to:    y-y1 = a*(x-x1) + b;

x = (1/a)*y + 1/a*(-y1-b+a*x1);

if x < x1 % line is on left side out of bounding box
    x = x1;
elseif x > x2 % line is on right side out of bounding box
    x = x2;
end

end

%% hough transform

function [xy_bb_top, xy_bb_bot] = calculate_topBot_lines(lines, y1, y3, x1, x2, debug)
% select best line for top edge and bottom edge from all lines detected
% input:    lines: from hough transform
%           x1, x2, y1, y3: bounding box coordinates
%           debug: boolean, if true prints detected lines in bounding box
%
% output:   xy_bb_top: coordinates of start and end point of best top line
%           xy_bb_bot: coordinates of start and end point of best left line
%           !!! coordinates start in bounding box top left corner with (0,0), 
%            not anymore in hole image !!!

% initialization
max_len_top = 0;
max_len_bot = 0;
line_of_interest = nan(length(lines),1);
xy_bb_top = 0;
xy_bb_bot = 0;


y_bb_1_3_top = floor((y3-y1)/3); % no + y1 due to the fact that this is in BB coords
y_bb_2_3_bot = floor(2*(y3-y1)/3);

for k = 1:length(lines)

    % only lines that are 45 or horizontal, not interessted in vertical
    line_of_interest(k) = 45 < abs(lines(k).theta) && abs(lines(k).theta) < 90;
    if line_of_interest(k) == true

        %line_top
        if lines(k).point1(2) < y_bb_1_3_top && lines(k).point2(2) < y_bb_1_3_top
            len = norm(lines(k).point1 - lines(k).point2);
            if (len > 1.1*max_len_top)
                max_len_top = len;
                % endpoint of longest top line
                xy_bb_top = [lines(k).point1; lines(k).point2];
            end
            % Debugging
            if debug == true
                figure(10)
                hold on
                plot([lines(k).point1(1);lines(k).point2(1)], [lines(k).point1(2);lines(k).point2(2)],'LineWidth',2);
            end
        %line_bottom
        elseif lines(k).point1(2) > y_bb_2_3_bot && lines(k).point2(2) > y_bb_2_3_bot
            len = norm(lines(k).point1 - lines(k).point2);
            if (len > 1.1*max_len_bot)
                max_len_bot = len;
                % endpoint of longest bot line
                xy_bb_bot = [lines(k).point1; lines(k).point2];
            end
            % Debuging
            if debug == true
                figure(10)
                hold on
                plot([lines(k).point1(1);lines(k).point2(1)], [lines(k).point1(2);lines(k).point2(2)],'LineWidth',2);
            end
        end
    end

end

end


function [xy_bb_left, xy_bb_right] = calculate_leftRight_lines(lines, x1, x2, y1, y3, debug)
% select best line for left edge and right edge from all lines detected
% input:    lines: from hough transform
%           x1, x2, y1, y3: bounding box coordinates
%           debug: boolean, if true prints detected lines in bounding box
%
% output:   xy_bb_left: coordinates of start and end point of best left line
%           xy_bb_right: coordinates of start and end point of best right line
%           !!! coordinates start in bounding box top left corner with (0,0), 
%            not anymore in hole image !!!

% initialization
max_len_left = 0;
max_len_right = 0;
line_of_interest = nan(length(lines),1);
xy_bb_left = 0;
xy_bb_right = 0;

% define borders for points of left and right lines to lie in
x_bb_2_3_left = floor(2*(x2-x1)/3); % pixel value at 2/3 of width
x_bb_1_3_right = floor((x2-x1)/3);  % pixel value at 1/3 of width

for k = 1:length(lines)

    % only lines that have an theta angle < 25° = "vertical lines"
    line_of_interest(k) = 0 < abs(lines(k).theta) && abs(lines(k).theta) < 25;
    if line_of_interest(k) == true

        %line_left
        if lines(k).point1(1) < x_bb_2_3_left && lines(k).point2(1) < x_bb_2_3_left
            len = norm(lines(k).point1 - lines(k).point2);
            if (len > 1.1*max_len_left) && (len > round(1/3*(y3-y1)))
                max_len_left = len;
                % endpoint of longest top line
                xy_bb_left = [lines(k).point1; lines(k).point2];
            end
            if debug == true
                figure(11)
                hold on
                plot([lines(k).point1(1);lines(k).point2(1)], [lines(k).point1(2);lines(k).point2(2)],'LineWidth',2);
            end
        %line_right
        elseif lines(k).point1(1) > x_bb_1_3_right && lines(k).point2(1) > x_bb_1_3_right
            len = norm(lines(k).point1 - lines(k).point2);
            if (len > 1.1*max_len_right) && (len > round(1/3*(y3-y1)))
                max_len_right = len;
                % endpoint of longest bot line
                xy_bb_right = [lines(k).point1; lines(k).point2];
            end
            if debug == true
                figure(11)
                hold on
                plot([lines(k).point1(1);lines(k).point2(1)], [lines(k).point1(2);lines(k).point2(2)],'LineWidth',2);
            end
        end
    end

end

end


function [y_top, y_bot, x_left, x_right] = holeEdgeShit(depth_img_woraster, x1, x2, y1, y3, bb_width, debug)
% return functions describing the frame along the selected edges
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
% output:   functions y=f(x) and x=f(y) defining the frame on which the
%           window frame is presumed in !!! image coordinates !!!

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

% 3. visualize the fitted lines
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

% 4. calculte best lines
% top and bot start and end point in bb coordinates !!!
[xy_bb_top, xy_bb_bot] = calculate_topBot_lines(lines, y1, y3, x1, x2, debug);
% left and right start and end point in bb coordinates !!!
[xy_bb_left, xy_bb_right] = calculate_leftRight_lines(lines, x1, x2, y1, y3, debug);

% 5. convert line to function of x coordinate
if x1-bb_width_half > 1
    x_ = x1-bb_width_half:x2+bb_width_half;
else
    x_ = x1:x2;
end
if y1-bb_width_half > 1
    y_ = y1-bb_width_half:y3+bb_width_half;
else
    y_ = y1:y3;
end
    
for x = x_
    if xy_bb_top ~= 0
        % calculate y coordinate from line function as y=f(x)
        y_top(x) = bbPoints2YEdgeBorder(x, xy_bb_top, y1, y3, x1);
    else
        % take bounding box limit
        y_top(x) = y1;
    end
    if xy_bb_bot ~= 0
        % calculate y coordinate from line function as y=f(x)
        y_bot(x) = bbPoints2YEdgeBorder(x, xy_bb_bot, y1, y3, x1);
    else
        % take bounding box limit
        y_bot(x) = y3;
    end
end

for y = y_
    if size(xy_bb_left,1) ~= 1 && ~(xy_bb_left(1,1)==xy_bb_left(2,1)) %2nd criterion is due to singularity of function y=f(x) can't map x=2
        % calculate x coordinate from line function as x=f(y)
        x_left(y) = bbPoints2XEdgeBorder(y, xy_bb_left, x1, x2, y1);
    else
        % take bounding box limit
        x_left(y) = x1;
    end
    if size(xy_bb_right,1) ~= 1 && ~(xy_bb_right(1,1)==xy_bb_right(2,1))
        % calculate x coordinate from line function as x=f(y)
        x_right(y) = bbPoints2XEdgeBorder(y, xy_bb_right, x1, x2, y1);
    else
        % take bounding box limit
        x_right(y) = x2;
    end
end

% round to integer values for use with pixels
y_top= round(y_top);
y_bot= round(y_bot);
x_left = round(x_left);
x_right= round(x_right);

end

%%

function [xs1, ys1, xs2,ys2, xs3,ys3, xs4,ys4] = getIntersectionEdges(x1, x2, y_top, y_bot, x_left, x_right)
% get intersections of edge function
% input:    x1,x2: left and riht bounding box borders
%           y_top, y_bot: functions for edge frame of x defining the y value
%           x_left, x_right: functions for edge frame of y defining the x value
% output:   coordinates of 4 intersection of edge frames inside bounding box
%           important is that these are already in image coordinates [px]

x_vec = x1:x2;

xs1 = find( x_left( y_top( x_vec ) ) == x_vec )+x1-1;
% no intersection on discrete pixel value
if size(xs1,2) == 0 
    xs1 = find( x_left( y_top( x_vec -1 ) ) == x_vec )+x1-1;
% no unambigous intersection (multiple)
elseif size(xs1,2) > 1
    xs1 = xs1(1);
end
ys1 = y_top(xs1);

xs2 = find( x_right( y_top( x_vec ) ) == x_vec )+x1-1;
if size(xs2,2) == 0 % xs4 ot found does not exist
    % intersection may is not in the pixel center
    xs2 = find( x_right( y_top( x_vec -1 ) ) == x_vec )+x1-1;
elseif size(xs2,2) > 1
    xs2 = xs2(1);
end
ys2 = y_top(xs2);

xs3 = find( x_right( y_bot( x_vec ) ) == x_vec )+x1-1;
if size(xs3,2) == 0 % xs4 ot found does not exist
    % intersection may is not in the pixel center
    xs3 = find( x_right( y_bot( x_vec -1 ) ) == x_vec )+x1-1;
elseif size(xs3,2) > 1
    xs3 = xs3(1);
end
ys3 = y_bot(xs3);

xs4 = find( x_left( y_bot( x_vec ) ) == x_vec )+x1-1;
if size(xs4,2) == 0 % xs4 ot found does not exist
    % intersection may is not in the pixel center
    xs4 = find( x_left( y_bot( x_vec-1 ) ) == x_vec )+x1-1;
elseif size(xs4,2) > 1
    xs4 = xs4(1);
end
ys4 = y_bot(xs4);
end

%% save images

function adjDepth2PngImage(depth_img_adj, label, rotate_back)
% save adjusted depth image as PNG image in path specified under filepath
%
% input:    depth_img_adj:  adjusted depth image in uint16 format
%           label:          number for labelling png image
%           rotate_back:    boolean, set true for rotate back depth images
%                           to overhead view -> needed for Open3D
%
% output:   none, png image is saved under filepath

% make changes here -------------------------------------------------------
type = 'depth_adj';
filepath =  '/home/andreas/Documents/ASL_window_dataset/depth_images_adj/temp/';
% -------------------------------------------------------------------------

if rotate_back == true
    img = imrotate(depth_img_adj, 180);
else
    img = depth_img_adj;
end

filename = strcat('asl_window_', num2str(label), '_', num2str(type), '.png');
imwrite(img, strcat(filepath, filename), 'fmt', 'png');

end

%% not used anymore 

function z = mapxy2plane(x, y, param)
% input:    x,y coordinates in pixel from origin at top left
% output:   mapped value of depth according to fitted plane
% z = ax + by + d

z = param(1)*x + param(2)*y + param(3);

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
load('data/');
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

