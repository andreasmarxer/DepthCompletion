clear; clc; close all;
%%=========================================================================
%% parameters

%%%%%%%%%%%%%%%%%%%
ploting = true;
rotate_back = true; % rotate the processed image before saving png
% images need to have corrected upright  view due to prediction bouding box format
debug = false;
save = false;
save_figures = false;
label = 210; %110
%%%%%%%%%%%%%%%%%%%

depth_procentual_th = 0.25; % [%] of depth measurements available
inlier_th = 50; % [mm] distance to plane that is considered as inlier
bb_width = 7; % must be odd number, width of frame around bb
inside = false; % if true also bb_width/2 inside the edge frame
                % if false only bb_width outside the edge frame

confidence_th = 95; % [%] above which prediction confidence consider window

area_th = 1/9; % procentage of area from bb that the triangle of chosen points must cover

% labels from original1 bag which were used for training network, not adapted and considered !
% asl_train_labels = [1,16,39,90,128,178,199,221,264,269,283,289,307,337,...
%                     350,355,361,363,365,368];          
asl_train_labels = [0]; % when training with dataset not used for training
                
if label == 0
    label_array = 2:1:516; 
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
        dataset = 'original5'; % loads the correct pose file and saving imgs
        path = '/home/andreas/Documents/ASL_window_dataset/original5/';
        depth_filename = strcat(path, 'depth_images_mm/' ,'asl_window_', 'depth_', num2str(label), '.png');
        depth_filename_median = strcat(path, 'depth_images_median5/','asl_window_', 'depth_median5_', num2str(label), '.png');
        bb_filename = strcat(path, 'rgb_images_predictions/' ,'asl_window_','rgb_', num2str(label), '.txt');
        rgb_filename = strcat(path, 'rgb_images/' ,'asl_window_', 'rgb_', num2str(label), '.jpg');
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
            title(strcat('Depth Image median filtered (k=5) - Frame : ', num2str(label)));
            % plots predicted bounding box corner points
            plot_bb(left_x_vec,top_y_vec,width_vec,height_vec,depth_img);
            pause(0.2) % otherwise sometimes this isn't plotted
        end
        
        %% rgb image
        rgb_img = imread(rgb_filename);
        
        if ploting == true
            figure(2)
            imshow(rgb_filename)
            title(strcat('RGB image - Frame : ', num2str(label)));
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
            % 1. returns line segments found in bounding box with Canny and Hough Transform
            % figure(9)
            lines_depth = getLinesInBB(depth_img, x1, x2, y1, y3, bb_width, debug);
            figure_id = 5;
            figure_title = 'all lines in depth image';
            plot_lines(figure_id, figure_title, x1, x2, y1, y3, lines_depth, debug, depth_img)
            
            lines_rgb = getLinesInBB(rgb_img, x1, x2, y1, y3, bb_width, debug);
            figure_id = 15;
            figure_title = 'all lines in rgb image';
            plot_lines(figure_id, figure_title, x1, x2, y1, y3, lines_rgb, debug, rgb_img)
            
            % 2. calculate angle to gravity vector and delete other lines
            [lines_depth, gravity_cam] = convert2cameraAlignedGravity(lines_depth, x1, y1, depth_img, label, dataset, debug);
            %%
            [lines_rgb, gravity_cam] = convert2cameraAlignedGravity(lines_rgb, x1, y1, depth_img, label, dataset, debug);

            % 3. delete all lines where we don't have depth measurments
            lines_depth = deleteLinesWoDepth(lines_depth, debug);
            figure_id = 6;
            figure_title = 'all line with available depth in depth image';
            plot_lines(figure_id, figure_title, x1, x2, y1, y3, lines_depth, debug, depth_img)
            
            lines_rgb = deleteLinesWoDepth(lines_rgb, debug);
            figure_id = 16;
            figure_title = 'all lines with available depth in rgb image';
            plot_lines(figure_id, figure_title, x1, x2, y1, y3, lines_rgb, debug, rgb_img)
            
            %% 4. delete all lines which dont are horizontal / vertical
            lines_depth = deleteNonHorizontalOrVerticalLines(lines_depth, debug);
            figure_id = 7;
            figure_title = 'all gravity lines with available depth in depth image';
            plot_lines(figure_id, figure_title, x1, x2, y1, y3, lines_depth, debug, depth_img)
            
            lines_rgb = deleteNonHorizontalOrVerticalLines(lines_rgb, debug);
            figure_id = 17;
            figure_title = 'all gravity lines with available depth in rgb image';
            plot_lines(figure_id, figure_title, x1, x2, y1, y3, lines_rgb, debug, rgb_img)
            
            lines = [lines_depth];
                        
%             % 3. select most best line 
%             % (most confident or next one when length is more than 10% longer)
%             % top and bot start and end point in bb coordinates !!!
%             [xy_bb_top, xy_bb_bot] = getTopBotLinesInBBwGravity(...
%                 lines, x1, x2, y1, y3, debug);
%             % left and right start and end point in bb coordinates !!!
%             [xy_bb_left, xy_bb_right] = getLeftRightLinesInBBwGravity(...
%                 lines, x1, x2, y1, y3, debug);
% 
%             % 4. convert line to function of x coordinate
%             [y_top, y_bot, x_left, x_right] = line2functionInBB(...
%                 x1, x2, y1, y3, xy_bb_top, xy_bb_bot, xy_bb_left,xy_bb_right,  bb_width, inside);
%             
%             % get intersections of edge function
%             [xs1, ys1, xs2,ys2, xs3,ys3, xs4,ys4] = getIntersectionEdges(...
%                 x1, x2, y_top, y_bot, x_left, x_right);
%             
            %% rgb image
            % plot edges in rgb image
%             if ploting == true
%                 figure(2)
%                 hold on
%                 plot(x1:x2, y_top(x1:x2))
%                 hold on
%                 plot(x_right(y2:y3), y2:y3)
%                 hold on
%                 plot(x1:x2, y_bot(x1:x2))
%                 hold on
%                 plot(x_left(y2:y3), y2:y3)
%                 hold on
%                 % plot all coordinates in bounding box edge frame
% %                 plot(points_x, points_y,'.');
% %                 hold on
%                 % plot intersection of edges in rgb
%                 plot([xs1;xs2;xs3;xs4],[ys1;ys2;ys3;ys4], 'y+');
%                 hold on
%             end
%             
%             % define frame width to take points for RANSAC
%             if inside == true
%                 % inside and outside bb_width_half
%                 bb_width_scale = -(bb_width-1)/2:1:(bb_width-1)/2;
%             else
%                 % only go outside the predicted bb for plane fitting
%                 bb_width_scale = 0:1:bb_width-1;
%             end   
            
%             %% 1. get all coordinates of points in bounding box edge frame
%             [points_x, points_y] = getEdgeframeCoordinates(xs1,ys1,...
%                 xs2,ys2, xs3,ys3, xs4,ys4, y_top, y_bot, x_left, x_right,...
%                 bb_width, bb_width_scale, depth_img);
%             
%             % initialize depth with NaNs           
%             points_z = NaN(size(points_x,1),1);
%             n_points_z_total = size(points_z,1);
% 
%             % assign depth measurment to z coordinate of plane
%             for n = 1:size(points_x,1)
%                 points_z(n) = depth_img(points_y(n),points_x(n));
%             end
% 
%             % only if more depth measurments than depth_th (%) in bb frame
%             if nnz(points_z) > depth_procentual_th * n_points_z_total
%                 % only consider the points where we have a depth measurement
%                 points_x = points_x(points_z>0);
%                 points_y = points_y(points_z>0);
%                 points_z = points_z(points_z>0); 
%                 % points_z must be the last otherwise this doesn't work!!!
%             else
%                 disp(strcat('Not enough depth points at window: ', num2str(window)));
%                 continue % with next window (for loop iteration)
%             end

%             %% 2. convert from image coordinates [px] to camera frame coordinates [mm]
%             
%             P_all = [points_x, points_y, points_z];
%             [points_X_c, points_Y_c, points_Z_c] = pixelNormalView2camCoordinate(P_all);
                            
            %% 3. make RANSAC in camera frame
            % RANSAC initialization
            best.first.random2 = zeros(1,2);
            best.second.random2 = zeros(1,2);
            best.third.random2 = zeros(1,2);
         
            best.first.param = zeros(1,3);
            best.second.param = zeros(1,3);
            best.third.param = zeros(1,3);
            
            best.first.inlier_array = zeros(length(lines),1);
            best.second.inlier_array = zeros(length(lines),1);
            best.third.inlier_array = zeros(length(lines),1);
            
            best.first.dist = 0;
            best.second.dist = 0;
            best.third.dist = 0;

            ransac_it_skipped = 0;
            while_count = 0;
            
            %% RANSAC ======================================================
            for ransac_it = 1:10 %500
                % initialize not unique!
                random2 = ones(1,2);

                % initialize (crit=1/true means we do pick again)
                must_ransac_crit = 1;
                opt_ransac_crit_area = 0;
                
                while_it = 1;
                n_tries_before_skip_ransac_it = 1000;
        
                % while loop ---------------------------------------------
                while must_ransac_crit || opt_ransac_crit_area
                    
                    % pick two random lines
                    random2 = randi([1 size(lines,2)],1,2);

                    % update the criterions that we have 3 different points
                    must_ransac_crit = size(unique(random2),2) ~= 2;
%                     || size(unique(points_X_c(random3)),1) < 2 || ...
%                         size(unique(points_Y_c(random3)),1) < 2;
                    
                    % update the criterion that we have a minimal area between the points
%                     [A_P_triangle, A_bb] = ...
%                         calculateAreas(random3, points_x, points_y, ...
%                         x1, x2, y2, y3); 
%                     opt_ransac_crit_area = A_P_triangle < area_th*A_bb;
                    
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
                %%
                % fit plane from 2 planes
                curr_lines = lines(random2);
                
                % first selected line
                points_x_c_1 = [curr_lines(1).point1_cam(1); curr_lines(1).point2_cam(1)];    
                points_y_c_1 = [curr_lines(1).point1_cam(2); curr_lines(1).point2_cam(2)];
                points_z_c_1 = [curr_lines(1).point1_cam(3); curr_lines(1).point2_cam(3)];
                % second selected line
                points_x_c_2 = [curr_lines(2).point1_cam(1); curr_lines(2).point2_cam(1)];
                points_y_c_2 = [curr_lines(2).point1_cam(2); curr_lines(2).point2_cam(2)];
                points_z_c_2 = [curr_lines(2).point1_cam(3); curr_lines(2).point2_cam(3)];

                % fit plane through 2 lines
                points_x = [points_x_c_1; points_x_c_2];
                points_y = [points_y_c_1; points_y_c_2];
                points_z = [points_z_c_1; points_z_c_2];
                param = [points_x, points_y, ones(size(points_x,1),1)] \ [points_z];
                
                % initialization
                points1_cam = zeros(length(lines), 3);
                points2_cam = zeros(length(lines), 3);
                
                % take all points to calculate distance
                for k=1:length(lines)
                    points1_cam(k,:) = [lines(k).point1_cam(1), lines(k).point1_cam(2), lines(k).point1_cam(3)];
                    points2_cam(k,:) = [lines(k).point2_cam(1), lines(k).point2_cam(2), lines(k).point2_cam(3)];
                end
                
                % calculate distance
                dist_1 = dist2plane(points1_cam, param);
                dist_2 = dist2plane(points2_cam, param);
                dist = sum(dist_1) + sum(dist_2);
                
                % thresholding
                dist_th = 100;
                inlier_array = (dist_1 < dist_th) & (dist_2 < dist_th);
                inliers = nnz(inlier_array);
                
                % save new best parameters of ransac
                if inliers > nnz(best.first.inlier_array)
                    %2->3, 1->2, insert 1 new
                    best.third = best.second;
                    best.second = best.first;
                    best.first.inlier_array = inlier_array;
                    best.first.dist = dist;
                    best.first.param = param;
                    best.first.random2 = random2;
                
                elseif inliers == nnz(best.first.inlier_array) && dist <= best.first.dist
                    if inlier_array == best.first.inlier_array
                        best.first.dist = dist;
                        best.first.param = param;
                        best.first.random2 = random2; 
                    else
                        best.third = best.second;
                        best.second = best.first;
                        best.first.inlier_array = inlier_array;
                        best.first.dist = dist;
                        best.first.param = param;
                        best.first.random2 = random2;
                    end
                    
                elseif (inliers == nnz(best.first.inlier_array) && dist > best.first.dist)
                    if (inliers >= nnz(best.second.inlier_array) && dist < best.second.dist)
                        best.third = best.second;
                        best.second.inlier_array = inlier_array;
                        best.second.dist = dist;
                        best.second.param = param;
                        best.second.random2 = random2;
                    end
                %    
                elseif inliers > nnz(best.second.inlier_array)
                    best.third = best.second;
                    best.second.inlier_array = inlier_array;
                    best.second.dist = dist;
                    best.second.param = param;
                    best.second.random2 = random2; 
                    
                elseif inliers == nnz(best.second.inlier_array) && dist <= best.second.dist
                    if inlier_array == best.second.inlier_array
                        best.second.dist = dist;
                        best.second.param = param;
                        best.second.random2 = random2; 
                    else
                        best.third = best.second;
                        best.second.inlier_array = inlier_array;
                        best.second.dist = dist;
                        best.second.param = param;
                        best.second.random2 = random2;
                    end
                    
                elseif (inliers == nnz(best.second.inlier_array) && dist > best.second.dist)
                    if (inliers >= nnz(best.third.inlier_array) && dist < best.third.dist)
                        best.third.inlier_array = inlier_array;
                        best.third.dist = dist;
                        best.third.param = param;
                        best.third.random2 = random2;
                    end
                    
                elseif inliers > nnz(best.third.inlier_array)
                    best.third.inlier_array = inlier_array;
                    best.third.dist = dist;
                    best.third.param = param;
                    best.third.random2 = random2;
                    
                elseif inliers == nnz(best.third.inlier_array) && dist <= best.third.dist
                    if inlier_array == best.third.inlier_array
                        best.third.dist = dist;
                        best.third.param = param;
                        best.third.random2 = random2; 
                    else
                        best.third.inlier_array = inlier_array;
                        best.third.dist = dist;
                        best.third.param = param;
                        best.third.random2 = random2;
                    end
                    
                end
                
                %% Debugging ==============================================
                if debug == true
                    figure(100)
                    close
                    figure(100)
                    plot3([0, gravity_cam(1)*1000], [0, gravity_cam(2)*1000], [0, gravity_cam(3)*1000], 'm')
                    xlabel('X_c')
                    ylabel('Y_c')
                    zlabel('Z_c')
                    xlim([-1500 1500])
                    ylim([-1500 1500])
                    zlim([0 2000])
                    
                    % plot all lines
                    for k = 1:length(lines)
                        hold on
                        plot3([points1_cam(k,1),points2_cam(k,1)], ...
                            [points1_cam(k,2),points2_cam(k,2)], [points1_cam(k,3), points2_cam(k,3)],'g');
                        scatter3([points1_cam(k,1),points2_cam(k,1)], ...
                            [points1_cam(k,2),points2_cam(k,2)], [points1_cam(k,3), points2_cam(k,3)], 'gx')
                    end
                    
                    % selected lines
                    hold on
                    plot3(points_x_c_1, points_y_c_1, points_z_c_1, 'b', 'LineWidth', 2)
                    hold on
                    scatter3(points_x_c_1, points_y_c_1, points_z_c_1, 'bx')

                    hold on
                    plot3(points_x_c_2, points_y_c_2, points_z_c_2, 'b', 'LineWidth', 2)
                    hold on
                    scatter3(points_x_c_2, points_y_c_2, points_z_c_2, 'bx')

                    % plane
                    [xx,yy] = meshgrid(-1500:100:1500,-1500:100:1500);
                    z = param(1)*xx + param(2)*yy + param(3);

                    surf(xx,yy,z, 'EdgeColor', 'black', 'FaceAlpha', 0.2)
                    title('Fitted plane from selected lines')
                   
                    pause(2)
                    
                    
                end
                
                %% ========================================================

            end
            %% end of ransac ==============================================
            
            if exist('best_random2', 'var')
                % save all considered points for plane fitting
                plane_pts = [plane_pts; P_all(best_random3,:)];
            end

            disp(strcat('Window: ', num2str(window), ' Best inlier ratio: ', num2str(best_inlier/size(points_z,1)), ' Best std: ', num2str(best_std)));
            disp(strcat('Iterations skipped : ', num2str(ransac_it_skipped)));
            
            % camera intrinsic with shifted principal point due to rotated image
            load('data/K_normalView.mat'); % loaded as K_normalView
            
            % adjust depth image for each pixel inside bounding box
            for x = x1:1:x2
                for y = y1:1:y3
                    % only adjust if we are also inside the chose window edges
                    if y_top(x)<=y && y_bot(x)>=y && x_left(y)<=x && x_right(y)>=x
                        
                        % convert x,y [px] to normalized image coordinates x,y [m] (z=1m)
                        [X_norm, Y_norm] = pixelNormalView2normImgCoordinates(x, y, K_normalView);

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
            pause(0.2) % otherwise sometimes this isn't plotted
            figure(3)
            pause(0.2) % otherwise sometimes this isn't plotted
            visualize_depth_png(depth_img_adj)
            pause(0.2) % otherwise sometimes this isn't plotted
            title(strcat('Completed depth image - Frame : ', num2str(label)));
            % plots predicted bounding box corner points
            plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);
            pause(0.1) % otherwise sometimes this isn't plotted
            if size(plane_pts,1) ~=0
                hold on
                plot(plane_pts(:,1),plane_pts(:,2),'mo')
                pause(0.5) % otherwise sometimes this isn't plotted
                figure(1)
                hold on
                plot(plane_pts(:,1),plane_pts(:,2),'mo')
                pause(0.1) % otherwise sometimes this isn't plotted
            end
            
        end

        %% save adjusted depth image as png
        if save_figures == true
            figure(1)
            saveas(gcf,strcat('figures/', dataset, '_depth_', num2str(label), '.png'))
            figure(2)
            saveas(gcf,strcat('figures/', dataset, '_rgb_', num2str(label), '.png'))
            figure(3)
            saveas(gcf,strcat('figures/', dataset, '_depth_completed_', num2str(label), '.png'))
        end
        if save == true            
            adjDepth2PngImage(depth_img_adj, label, dataset, rotate_back)
            pause(0.2); % otherwise some images don't get saved
        end
        
        toc;
        
    end % end of else
end % end of label for loop