clear; clc; close all;
%%=========================================================================
%% parameters

%%%%%%%%%%%%%%%%%%%
ploting = false;
rotate_back = true; % rotate the processed image before saving png
% images need to have corrected upright  view due to prediction bouding box format
debug = false;
save = true;
save_figures = false;
label = 0; %110
%%%%%%%%%%%%%%%%%%%

confidence_th = 95; % [%] above which prediction confidence consider window
dist_th = 50;
% not used anymore !!!
bb_width = 7;
inside = false;

% labels from original1 bag which were used for training network, not adapted and considered !
% asl_train_labels = [1,16,39,90,128,178,199,221,264,269,283,289,307,337,...
%                     350,355,361,363,365,368];          
asl_train_labels = [0]; % when training with dataset not used for training
                
if label == 0
    label_array = 2:1:496; 
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
   
        %% rgb image
        rgb_img = imread(rgb_filename);
        
        %% median filtered (kernel 5x5) depth image
        depth_img = imread(depth_filename_median); % meters

        %% adjust the depth image
        % initialization
        depth_img_adj = depth_img;  % finally contains depth measurment and fitted planes
        depth_img_corr = uint16(zeros(size(depth_img))); % only contains fitted planes
        plane_pts = uint16([]); % points chosen to fit plane for all predictions
        
        %% VISUALISATION
        if ploting == true
                figure(1);
                close;
                figure(1);
                set(gcf, 'Position', [437 136 1986 1193]);
                set(gcf, 'Units', 'pixels');

                %% RGB image
                % left bottom width height
                subplot('Position', [0.015, 0, 0.3, 1])
                imshow(rgb_filename)
                title(strcat('\fontsize{20}RGB image - Frame : ', num2str(label)));
                % plots predicted bounding box corner points
                plot_bb(left_x_vec,top_y_vec,width_vec,height_vec,depth_img);

                %% DEPTH FILTERED
                subplot('Position', [1/3-0.005, 0, 0.3, 1])
                visualize_depth_png(depth_img)  % millimeters
                title(strcat('\fontsize{20}Filtered Depth Image (k=5) - Frame : ', num2str(label)));
                % plots predicted bounding box corner points
                plot_bb(left_x_vec,top_y_vec,width_vec,height_vec,depth_img);

                %% DEPTH COMPLETED
                subplot('Position', [2/3-0.005, 0, 0.3, 1])
                visualize_depth_png(depth_img_adj)  % millimeters
                title(strcat('\fontsize{20}Completed Depth Image - Frame : ', num2str(label)));
                % plots predicted bounding box corner points
                plot_bb(left_x_vec,top_y_vec,width_vec,height_vec, depth_img);
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% for every prediction in this image
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
            pause(0.2);
            
            lines_rgb = getLinesInBB(rgb_img, x1, x2, y1, y3, bb_width, debug);
            figure_id = 15;
            figure_title = 'all lines in rgb image';
            plot_lines(figure_id, figure_title, x1, x2, y1, y3, lines_rgb, debug, rgb_img)
            pause(0.2);
            
            % 2. calculate angle to gravity vector and delete other lines
            [lines_depth, ~] = convert2cameraAlignedGravity(lines_depth, x1, y1, depth_img, label, dataset, debug);
            [lines_rgb, gravity_cam] = convert2cameraAlignedGravity(lines_rgb, x1, y1, depth_img, label, dataset, debug);

            % 3. delete all lines where we don't have depth measurments
            lines_depth = deleteLinesWoDepth(lines_depth, debug);
            figure_id = 6;
            figure_title = 'all line with available depth in depth image';
            plot_lines(figure_id, figure_title, x1, x2, y1, y3, lines_depth, debug, depth_img)
            pause(0.2);
            
            lines_rgb = deleteLinesWoDepth(lines_rgb, debug);
            figure_id = 16;
            figure_title = 'all lines with available depth in rgb image';
            plot_lines(figure_id, figure_title, x1, x2, y1, y3, lines_rgb, debug, rgb_img)
            pause(0.2);
            
            % 4. delete all lines which dont are horizontal / vertical
            lines_depth = deleteNonHorizontalOrVerticalLines(lines_depth, debug);
            figure_id = 7;
            figure_title = 'all gravity lines with available depth in depth image';
            plot_lines(figure_id, figure_title, x1, x2, y1, y3, lines_depth, debug, depth_img)
            pause(0.2);
            
            lines_rgb = deleteNonHorizontalOrVerticalLines(lines_rgb, debug);
            figure_id = 17;
            figure_title = 'all gravity lines with available depth in rgb image';
            plot_lines(figure_id, figure_title, x1, x2, y1, y3, lines_rgb, debug, rgb_img)
            pause(0.2);
            
            if size(lines_depth,2)~=0 && size(lines_rgb,2)~=0
                lines = [lines_depth, lines_rgb];
            elseif size(lines_depth,2)~=0 && size(lines_rgb,2)==0
                lines = lines_depth;
            elseif size(lines_depth,2)==0 && size(lines_rgb,2)~=0
                lines = lines_rgb;
            else
                disp('No lines detected');
                break;
            end
              
            figure_id = 11;
            figure_title = 'all gravity lines with available depth in BOTH image';
            plot_lines(figure_id, figure_title, x1, x2, y1, y3, lines, debug, rgb_img)
            pause(0.2);
            
            %% make RANSAC in camera frame
            % RANSAC initialization
            clear best;
            best.first.random2 = zeros(1,2);
            best.second.random2 = zeros(1,2);
            best.third.random2 = zeros(1,2);
         
            best.first.param = zeros(1,3);
            best.second.param = zeros(1,3);
            best.third.param = zeros(1,3);
            
            best.first.inlier_array = false(length(lines),1);
            best.second.inlier_array = false(length(lines),1);
            best.third.inlier_array = false(length(lines),1);
            
            best.first.dist = Inf;
            best.second.dist = Inf;
            best.third.dist = Inf;

            ransac_it_skipped = 0;
            while_count = 0;
            
            % initialization
            points1_cam = zeros(length(lines), 3);
            points2_cam = zeros(length(lines), 3);

            % take all points to calculate distance
            for k=1:length(lines)
                points1_cam(k,:) = [lines(k).point1_cam(1), lines(k).point1_cam(2), lines(k).point1_cam(3)];
                points2_cam(k,:) = [lines(k).point2_cam(1), lines(k).point2_cam(2), lines(k).point2_cam(3)];
            end
                
            %% RANSAC ======================================================
            for ransac_it = 1:500 
                % initialize not unique!
                random2 = ones(1,2);

                % initialize (crit=1/true means we do pick again)
                must_ransac_crit = 1;
                opt_ransac_crit = 1;
                
                while_it = 1;
                n_tries_before_skip_ransac_it = 1000;
        
                % while loop ---------------------------------------------
                while must_ransac_crit || opt_ransac_crit
                    
                    % pick two random lines
                    random2 = randi([1 size(lines,2)],1,2);

                    % update the criterions that we have 3 different points
                    must_ransac_crit = size(unique(random2),2) ~= 2;
                    
                    % check if the selected lines are perpendicular
                    angle_diff = abs(lines(random2(1)).angle2gravity - lines(random2(2)).angle2gravity);
                    % if the angle difference is smaller than 75°, not perpendicular, 
                    % remember that we only have ~90° or ~0° angles +- 5°
                    opt_ransac_crit = angle_diff < 10; 
                    
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
                
                %% fit plane from 2 planes
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
                
                % calculate distance
                dist_1 = dist2plane(points1_cam, param);
                dist_2 = dist2plane(points2_cam, param);
                dist = sum(dist_1) + sum(dist_2);
                
                % thresholding
                inlier_array = (dist_1 < dist_th) & (dist_2 < dist_th);
                inliers = nnz(inlier_array);
                %disp(strcat('inlier_array  : ',num2str(inlier_array')));
                %disp(strcat('inliers       : ', num2str(inliers)));
                %disp('---------------------------------------------');

                % save new best parameters of ransac
                random2_variants = [random2 ; random2(2), random2(1)];
                random2_array = repmat(random2_variants,3,1);
                
                best2_array = [repmat(best.first.random2,2,1);
                              repmat(best.second.random2,2,1);
                              repmat(best.third.random2,2,1)];
                          
                check_if_already_in = (random2_array == best2_array);
                check = check_if_already_in(:,1) & check_if_already_in(:,2);
                          
                if nnz(check) > 0
                    %disp('already in best candidates, skipt this iteration')
                    continue;
                    
                elseif inliers > nnz(best.first.inlier_array)
                    best.third = best.second;
                    best.second = best.first;
                    best.first.inlier_array = inlier_array;
                    best.first.dist = dist;
                    best.first.param = param;
                    best.first.random2 = random2;
                
                elseif inlier_array == best.first.inlier_array
                    if (dist <= best.first.dist)
                        best.first.dist = dist;
                        best.first.param = param;
                        best.first.random2 = random2; 
                    end
                   
                elseif (inliers == nnz(best.first.inlier_array)) && (dist < best.first.dist)
                    % can't be the same inlier array as first due to previous elseif
                    if inlier_array ~= best.second.inlier_array
                        best.third = best.second;
                        best.second = best.first;
                        best.first.inlier_array = inlier_array;
                        best.first.dist = dist;
                        best.first.param = param;
                        best.first.random2 = random2;
                    elseif inlier_array == best.second.inlier_array
                        % delete best.second, best.third stays
                        best.second = best.first;
                        best.first.inlier_array = inlier_array;
                        best.first.dist = dist;
                        best.first.param = param;
                        best.first.random2 = random2;
                    end
                % second  
                elseif inliers > nnz(best.second.inlier_array)
                    best.third = best.second;
                    best.second.inlier_array = inlier_array;
                    best.second.dist = dist;
                    best.second.param = param;
                    best.second.random2 = random2;
                    
                elseif inlier_array == best.second.inlier_array
                    if (dist <= best.second.dist)
                        best.second.dist = dist;
                        best.second.param = param;
                        best.second.random2 = random2; 
                    end
                    
                elseif (inliers == nnz(best.second.inlier_array)) && (dist < best.second.dist)
                    % can't be the same inlier array as first or second due to previous elseif
                    % BUT it could be the same as the third, this can be neglected 
                    % due to the fact that the third is anyway deleted!!!!
                    best.third = best.second;
                    best.second.inlier_array = inlier_array;
                    best.second.dist = dist;
                    best.second.param = param;
                    best.second.random2 = random2;
                    
                % third    
                elseif inliers > nnz(best.third.inlier_array)
                    best.third.inlier_array = inlier_array;
                    best.third.dist = dist;
                    best.third.param = param;
                    best.third.random2 = random2;
                    
                elseif inlier_array == best.third.inlier_array
                    if (dist <= best.third.dist)
                        best.third.dist = dist;
                        best.third.param = param;
                        best.third.random2 = random2; 
                    end
                    
                elseif (inliers == nnz(best.third.inlier_array)) && (dist < best.third.dist)
                        best.third.inlier_array = inlier_array;
                        best.third.dist = dist;
                        best.third.param = param;
                        best.third.random2 = random2; 
                    
                end
                
            end
            %% end of ransac ==============================================
            
            if best.first.random2 == [0,0];
                disp(strcat('Skipped window : ', num2str(window), ' no plane fitted'));
                continue;
            end
            %% project back to depth image and select a line for top/bot etc.
            spanned_area_in_px = zeros(4,1);
            frame_length = zeros(4,1);
            
            for plane = 1:4
                if plane == 1
                    selection = best.first.inlier_array;
                    best_param = best.first.param;
                elseif plane == 2
                    selection = best.second.inlier_array;
                    best_param = best.second.param;
                elseif plane == 3
                    selection = best.third.inlier_array;
                    best_param = best.third.param;
                elseif plane == 4
                    selection_plane = find(max(spanned_area_in_px)==spanned_area_in_px);
                    %selection_plane = find(max(frame_length)==frame_length);
                    
                    if selection_plane == 1
                        selection = best.first.inlier_array;
                        best_param = best.first.param;
                    elseif selection_plane == 2
                        selection = best.second.inlier_array;
                        best_param = best.second.param;
                    elseif selection_plane == 3
                        selection = best.third.inlier_array;
                        best_param = best.third.param; 
                    end                        
                end

                sel_lines = lines(selection);

                pause(0.2);
                figure(8)
                close;
                figure_id = 8;
                figure_title = strcat('all selected lines from plane :', num2str(plane));
                plot_lines(figure_id, figure_title, x1, x2, y1, y3, sel_lines, debug, depth_img)
                pause(1);

                % top and bot start and end point in bb coordinates !!!
                [xy_bb_top, xy_bb_bot] = getTopBotLinesInBBwGravity_2(...
                    sel_lines, x1, x2, y1, y3, depth_img, debug); %figure(9)

                % left and right start and end point in bb coordinates !!!
                [xy_bb_left, xy_bb_right] = getLeftRightLinesInBBwGravity_2(...
                    sel_lines, x1, x2, y1, y3, depth_img, debug); %figure(10)
 
                x_all = [xy_bb_top(:,1); xy_bb_bot(:,1); xy_bb_left(:,1); xy_bb_right(:,1)];
                y_all = [xy_bb_top(:,2); xy_bb_bot(:,2); xy_bb_left(:,2); xy_bb_right(:,2)];
                
                spanned_area_in_px(plane) = (max(x_all)-min(x_all))*(max(y_all)-min(y_all));
                
                frame_length(plane) = norm(xy_bb_bot(1,:)-xy_bb_bot(2,:))+norm(xy_bb_top(1,:)-xy_bb_top(2,:))...
                    + norm(xy_bb_left(1,:)-xy_bb_left(2,:))+norm(xy_bb_right(1,:)-xy_bb_right(2,:));
                
            end
            
%              %% Debugging ==============================================
%             if debug == true
%                 figure(100)
%                 plot3([0, gravity_cam(1)*1000], [0, gravity_cam(2)*1000], [0, gravity_cam(3)*1000], 'm')
%                 xlabel('X_c')
%                 ylabel('Y_c')
%                 zlabel('Z_c')
%                 xlim([-3000 3000])
%                 ylim([-3000 3000])
%                 zlim([0 3000])
% 
%                 % plot all lines
%                 for k = 1:length(lines)
%                     hold on
%                     if selection(k)==1
%                         color = 'r';
%                     else
%                         color = 'g';
%                     end
%                     
%                     plot3([points1_cam(k,1),points2_cam(k,1)], ...
%                         [points1_cam(k,2),points2_cam(k,2)], [points1_cam(k,3), points2_cam(k,3)], color);
%                     scatter3([points1_cam(k,1),points2_cam(k,1)], ...
%                         [points1_cam(k,2),points2_cam(k,2)], [points1_cam(k,3), points2_cam(k,3)], 'gx')
%                 end
% 
%                 % best lines
%                 best_lines = lines(best.first.random2);  %%%%%%%%%%%%%%
%                 
%                 % first selected line
%                 points_x_c_1 = [best_lines(1).point1_cam(1); best_lines(1).point2_cam(1)];    
%                 points_y_c_1 = [best_lines(1).point1_cam(2); best_lines(1).point2_cam(2)];
%                 points_z_c_1 = [best_lines(1).point1_cam(3); best_lines(1).point2_cam(3)];
%                 % second selected line
%                 points_x_c_2 = [best_lines(2).point1_cam(1); best_lines(2).point2_cam(1)];
%                 points_y_c_2 = [best_lines(2).point1_cam(2); best_lines(2).point2_cam(2)];
%                 points_z_c_2 = [best_lines(2).point1_cam(3); best_lines(2).point2_cam(3)];
% 
%                 hold on
%                 plot3(points_x_c_1, points_y_c_1, points_z_c_1, 'b', 'LineWidth', 2)
%                 hold on
%                 scatter3(points_x_c_1, points_y_c_1, points_z_c_1, 'bx')
% 
%                 hold on
%                 plot3(points_x_c_2, points_y_c_2, points_z_c_2, 'b', 'LineWidth', 2)
%                 hold on
%                 scatter3(points_x_c_2, points_y_c_2, points_z_c_2, 'bx')
% 
%             end
            
            
            %% calculate new bounding box
            % convert lines to function of other coordinate 
            % limiting the bounding box
            [y_top, y_bot, x_left, x_right] = line2functionInBB(...
                x1, x2, y1, y3, xy_bb_top, xy_bb_bot, xy_bb_left, xy_bb_right,  bb_width, inside);
            
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

                        % if we have a crappy plane ignore it
                        if new_value < 1
                            continue;
                        % use the new value for every pixel if no depth is available
                        elseif depth_img(y,x) == 0 && depth_img_corr(y,x) == 0
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
        
            %% VISUALIZATION
            if ploting == true
                figure(1);
                hold on
                %% RGB image
                % left bottom width height
                subplot('Position', [0.015, 0, 0.3, 1])
                % plot edges in rgb image
                hold on
                plot(x1:x2, y_top(x1:x2))
                hold on
                plot(x_right(y2:y3), y2:y3)
                hold on
                plot(x1:x2, y_bot(x1:x2))
                hold on
                plot(x_left(y2:y3), y2:y3)
                hold on

                %% DEPTH COMPLETED
                subplot('Position', [2/3-0.005, 0, 0.3, 1])
                hold on
                visualize_depth_png(depth_img_adj)  % millimeters
                title(strcat('\fontsize{20}Completed Depth Image - Frame : ', num2str(label)));
                % plots predicted bounding box corner points
                plot_bb(left_x_vec,top_y_vec,width_vec,height_vec, depth_img);

            end
        end 
        % end of window for loop
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
        %% save adjusted depth image as png
        if save_figures == true
            figure(1)
            saveas(gcf,strcat('figures/', dataset, '_all3_', num2str(label), '.png'))
        end
        if save == true            
            adjDepth2PngImage(depth_img_adj, label, dataset, rotate_back)
            pause(0.2); % otherwise some images don't get saved
        end
        
        toc;
        
    end % end of else
    
end % end of label for loop