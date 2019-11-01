clear; clc; close all;
%%=========================================================================
filename = '/home/andreas/Documents/ros_recording/2019-10-15-17-48-49.bag';
bag = rosbag(filename);

%% params
frame = 2;
rotation = true;
print = false;
save_img = false;
%%

topic = 'medium_resolution/depth_registered/image';
topic_bag = select(bag, 'Topic', topic);

topic_frame_msg_cell = readMessages(topic_bag, frame); % take image message of frames of bag
topic_frame_msg = topic_frame_msg_cell{1}; % extract from cell
img = readImage(topic_frame_msg);   % 480x640 single, with NaNs, [m] % !!!!!!!!!

if rotation == true
    img = imrotate(img, 180);
end

if print == true
    figure(1)
    imshow(img);
end

if save_img == true
        filename = strcat('asl_window_', num2str(type), '_', num2str(label), '.png');
        img(isnan(img))=0;
        convert from m to mm
        img = img * 1000;  
        convert to uint16
        img_uint16 = uint16(img);
        imwrite(img_uint16, filename, 'fmt', 'png');
end 

% PNG image: uint16, [mm]