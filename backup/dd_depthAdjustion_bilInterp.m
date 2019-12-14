% depth adjustion with bilinear interpolation approach

clear; clc; close all;

label = 370;
kernel = 5;
kernel_half = (kernel-1)/2;
    
asl_train_labels = [1,16,39,90,128,178,199,221,264,269,283,289,307,337,350,355,361,363,365,368];
if ismember(label, asl_train_labels)
    error('ERROR: This image was used for training!');
end

path = '/home/andreas/Documents/ASL_window_dataset/';
depth_filename = strcat(path, 'depth_images_mm/' ,'asl_window_', num2str(label), '_depth', '.png');
bb_filename = strcat(path, 'rgb_images_predictions/' ,'asl_window_', num2str(label), '_rgb', '.txt');

[class, confidence, left_x_vec, top_y_vec, width_vec, height_vec] = importBoundingBoxes(bb_filename);

%% original depth image
depth_img = imread(depth_filename); % meters

figure(1)
visualize_depth_png(depth_img)  % millimeters
title(strcat('Original Depth Image - Frame:', num2str(label)));
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

    f1 = kernelMedian(kernel, x1, y1, depth_img);  %takes median of 3x3 arround pixel neglecting zeros
    f2 = kernelMedian(kernel, x2, y2, depth_img);  %takes median of 3x3 arround pixel neglecting zeros 
    f3 = kernelMedian(kernel, x3, y3, depth_img);  %takes median of 3x3 arround pixel neglecting zeros 
    f4 = kernelMedian(kernel, x4, y4, depth_img);  %takes median of 3x3 arround pixel neglecting zeros 

    %% don't interpolate if one of the corners is zero
    if f1==0 || f2==0 || f3==0 || f4==0
        disp(strcat('NO INTERPOLATION: Dont have 4 corners with depth! @ window # ', num2str(window)));
        continue
    end

    %% else bilinear interpolation inside window bounding box
    p1 = [double(x1), double(y1), double(f1)];
    p2 = [double(x2), double(y2), double(f2)];
    p3 = [double(x3), double(y3), double(f3)];
    p4 = [double(x4), double(y4), double(f4)];
    for x = x1:1:x2
        for y = y1:1:y4
            depth_img_corr(y,x) = bilinInterpBoundingBox(x, y, p1, p2, p3, p4);
            % only use it if nothing is in front of corrected window depth
            if depth_img(y,x) == 0 || depth_img_corr(y,x)<depth_img(y,x)
                depth_img_adj(y,x) = depth_img_corr(y,x);
            end
        end
    end
end    

figure(3)
visualize_depth_png(depth_img_corr)  % millimeters
title(strcat('Bilinear interpolated windows - Frame : ', num2str(label)));
plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);
pause(0.2)

figure(4)
visualize_depth_png(depth_img_adj)
title(strcat('Corrected depth image (k=', num2str(kernel), ') with bilinear interpolated windows - Frame : ', num2str(label)));
plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);
pause(1)

%%
rgb_filename = strcat(path, 'rgb_images/' ,'asl_window_', num2str(label), '_rgb', '.jpg');

figure(5)
imshow(rgb_filename)
title(strcat('RGB image - Frame : ', num2str(label)));
plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);

