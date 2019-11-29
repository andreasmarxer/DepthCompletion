clear; clc; close all;
%%=========================================================================
%% parameters

%%%%%%%%%%%%%%%%%%%
ploting = true;
rotate_back = true; % rotate the processed image before saving png
% images need to have corrected upright  view due to prediction bouding box format
debug = true;
save = false;
label = 126;
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

            % 1. returns line segments found in bounding box with Canny and Hough Transform
            lines = getLinesInBB(depth_img, x1, x2, y1, y3, bb_width, debug);
            
            %% gravity check
%             for k = 1:length(lines)
%                 initialization
%                 points_z = zeros(2,1);
%                 kernel = 5;
%                 
%                 while any(points_z == 0)
%                     points_x = [lines(k).point1(1); lines(k).point2(1)];
%                     points_x = points_x + x1;
%                     points_y = [lines(k).point1(2); lines(k).point2(2)];
%                     points_y = points_y + y1;
%                     points_z = [kernelMedian(kernel, points_x(1), points_y(1), depth_img);
%                                 kernelMedian(kernel, points_x(2), points_y(2), depth_img)];
%                     if any(points_z == 0)
%                         kernel = kernel + 1;
%                         disp(strcat('Increased kernel: ', num2str(kernel)));
%                     end
%                 end
%                 figure(2)
%                 hold on
%                 plot(points_x, points_y,'LineWidth',2);            
%                 
%                 P_line = [points_x, points_y, points_z];
%                 [X_c_line, Y_c_line, Z_c_line] = pixelNormalView2camCoordinate(P_line);
%                 P_c_line = [X_c_line'; Y_c_line'; Z_c_line'];
%                 load('data/poses_lookedUp.mat'); % loaded as poses
%                 convert translation from [m] to [mm]
%                 poses(1:3,4,label) = poses(1:3,4,label)*1000;
%                 T_wc = poses(:,:,label);
% 
%                 P =   T_wc * [P_c_line; ones(1,size(P_c_line,2))];
%                 P_w_line = P(1:end-1,:);
%                 
%                 vectors
%                 V_line = P_w_line(:,1)-P_w_line(:,2);
%                 V_z = [0;0;1];
%                 angle2zAxis = rad2deg(acos( (V_line'*V_z) / (norm(V_line))))
%                 lines(k).angle2z = angle2zAxis;
% %                 figure(5); 
% %                 scatter3(P_w_line(1,:), P_w_line(2,:), P_w_line(3,:) ,'x')
% %                 xlabel('X_w')
% %                 ylabel('Y_w')
% %                 zlabel('Z_w')
% 
%                 if lines(k).angle2z < 2
%                     figure(13)
%                     hold on
%                     plot([lines(k).point1(1);lines(k).point2(1)], [lines(k).point1(2);lines(k).point2(2)],'LineWidth',2);
%                 elseif lines(k).angle2z < 92 && lines(k).angle2z > 88
%                     figure(14)
%                     hold on
%                     plot([lines(k).point1(1);lines(k).point2(1)], [lines(k).point1(2);lines(k).point2(2)],'LineWidth',2);
%                 end
%             end
            
            % 2. calculte best lines
            % top and bot start and end point in bb coordinates !!!
            [xy_bb_top, xy_bb_bot] = getTopBotLinesInBBwGravity(...
                lines, y1, y3, x1, x2, depth_img, label, debug);
            % left and right start and end point in bb coordinates !!!
            [xy_bb_left, xy_bb_right] = getLeftRightLinesInBBwGravity(...
                lines, x1, x2, y1, y3, depth_img, label, debug);

            % 3. convert line to function of x coordinate
            [y_top, y_bot, x_left, x_right] = line2functionInBB(...
                x1, x2, y1, y3, xy_bb_top, xy_bb_bot, xy_bb_left,xy_bb_right,  bb_width);
            
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
            
            P_all = [points_x, points_y, points_z];
            [points_X_c, points_Y_c, points_Z_c] = pixelNormalView2camCoordinate(P_all);
                            
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

            % adjust depth image for each pixel inside bounding box
            for x = x1:1:x2
                for y = y1:1:y3
                    % only adjust if we are also inside the chose window edges
                    if y_top(x)<=y && y_bot(x)>=y && x_left(y)<=x && x_right(y)>=x
                        
                        % convert x,y [px] to normalized image coordinates x,y [m] (z=1m)
                        [X_norm, Y_norm] = pixelNormalView2normImgCoordinates(x, y);

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
            title(strcat('Corrected depth image (k=5) with RANSAC plane fitted windows - Frame : ', num2str(label)));
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
        if save == true            
            adjDepth2PngImage(depth_img_adj, label, rotate_back)
            pause(0.2); % otherwise some images don't get saved
        end
        
        toc;
        
    end % end of else
end % end of label for loop