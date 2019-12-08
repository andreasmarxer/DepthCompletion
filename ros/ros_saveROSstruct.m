clear; clc; close all;
%%=========================================================================

asl_train_labels = [1,16,39,90,128,178,199,221,264,269,283,289,307,337,350,355,361,363,365,368];

for label = 1:1:488
    
    if ismember(label, asl_train_labels)
        continue;
    else
        disp(strcat('-Label : ', num2str(label)));
        tic;
        
        %% fix dependencies

        path = '/home/andreas/Documents/ASL_window_dataset/';
        depth_filename = strcat(path, 'depth_images_mm/' ,'asl_window_', num2str(label), '_depth', '.png');
        bb_filename = strcat(path, 'rgb_images_predictions/' ,'asl_window_', num2str(label), '_rgb', '.txt');
        [~, ~, left_x_vec, top_y_vec, width_vec, height_vec] = importBoundingBoxes(bb_filename);

        %% original depth image
        depth_img = imread(depth_filename); % meters

        %% save struct 1
        load('/home/andreas/Documents/MATLAB/time_corresp.mat');
        
        struct(label).time_stamp = time_corresp(label);
        num_pred = size(left_x_vec,1);
        struct(label).num_pred = num_pred;
        struct(label).x1 = zeros(num_pred,1);
        struct(label).x2 = zeros(num_pred,1);
        struct(label).y1 = zeros(num_pred,1);
        struct(label).y3 = zeros(num_pred,1);
        
        
        for window = 1:num_pred
            
            [x1, y1, x2, ~, ~, y3, ~, ~ ] = convertToBBCoords(left_x_vec(window),...
                top_y_vec(window), width_vec(window), height_vec(window), depth_img);

            struct(label).x1(window) = x1;
            struct(label).x2(window) = x2;
            struct(label).y1(window) = y1;
            struct(label).y3(window) = y3;
 
        end
        
    end % end of else
end