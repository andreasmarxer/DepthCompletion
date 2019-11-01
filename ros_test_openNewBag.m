clear; clc; close all;
%% =========================================================================
filename = '/home/andreas/2019-11-01-16-02-47_fixed.bag';
bag = rosbag(filename);
bagInfo = rosbag('info',filename);

%topic = '/depth_adapted/image';
topic = '/binary_predictions/image';
frame = 1;

topic_bag = select(bag, 'Topic', topic);
topic_frame_msg_cell = readMessages(topic_bag, frame); % take image message of frames of bag
topic_frame_msg = topic_frame_msg_cell{1}; % extract from cell
img = readImage(topic_frame_msg);
hold on
imshow(img);

%% fix dependencies

label = 2;

path = '/home/andreas/Documents/ASL_window_dataset/';
depth_filename = strcat(path, 'depth_images_mm/' ,'asl_window_', num2str(label), '_depth', '.png');
bb_filename = strcat(path, 'rgb_images_predictions/' ,'asl_window_', num2str(label), '_rgb', '.txt');
rgb_filename = strcat(path, 'rgb_images/' ,'asl_window_', num2str(label), '_rgb', '.jpg');
[class, confidence, left_x_vec, top_y_vec, width_vec, height_vec] = importBoundingBoxes(bb_filename);

%% original depth image
depth_img = imread(depth_filename); % meters

% visualize_depth_png(depth_img)  % millimeters
% title(strcat('Original Depth Image - Frame:', num2str(label)));
plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, depth_img);
pause(0.2) % otherwise sometimes this isn't plotted
