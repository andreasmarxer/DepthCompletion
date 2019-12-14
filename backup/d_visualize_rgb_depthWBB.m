clear; clc; close all;


for label = 1:1:488
    
    asl_train_labels = [1,16,39,90,128,178,199,221,264,269,283,289,307,337,350,355,361,363,365,368];
    if ismember(label, asl_train_labels)
        continue %label was used in training, don't show and skip iteration
    end

    path = '/home/andreas/Documents/ASL_window_dataset/';
    rgb_filename = strcat(path, 'rgb_images_predictions/' ,'asl_window_', num2str(label), '_rgb', '.jpg');
    depth_filename = strcat(path, 'depth_images_mm/' ,'asl_window_', num2str(label), '_depth', '.png');
    bb_filename = strcat(path, 'rgb_images_predictions/' ,'asl_window_', num2str(label), '_rgb', '.txt');

    figure(1)

    %% rgb image
    subplot(1,2,1)
    imshow(rgb_filename)
    title('RGB image with YOLOv3 prediction')

    %% depth image
    depth_img = imread(depth_filename);
    subplot(1,2,2)
    visualize_depth_png(depth_img)
    [class, confidence, left_x, top_y, width, height] = importBoundingBoxes(bb_filename);

    for window = 1:size(left_x)
        [x1, y1, x2, y2, x3, y3, x4, y4 ] = plot_bb(left_x(window), top_y(window), width(window), height(window), depth_img);
    end

    %% figure properties
    set(gcf, 'Units', 'normalized', 'Position', [0.1,0.1,0.8,0.8] ) ;
    set(gcf,'name',strcat('RGB Depth Image Viewer - Frame : ', num2str(label)), 'numbertitle','off');
    
w = waitforbuttonpress;

end

%% functions
function visualize_depth_png(img_png)
    img_scaled = 10*img_png;    %scale that uint16 range is better used
    imshow(img_scaled, 'colormap', jet);
    originalSize = get(gca, 'Position');
    c = colorbar;
    c.Label.String = 'Depth in [10⁻4m]';
    title('CamBoard pico monstar depth measurement')
    set(gca, 'Position', originalSize); % set current axis to size before colormap
    
end

