%clear; clc; close all;
%%=========================================================================
filename = '/media/andreas/SanDiskSSD/Semester_Thesis/ros_recording/2019-10-15-17-48-49_original.bag';
bag = rosbag(filename);

%% params
frame = 1;
rotation = true;
print = false;
save_img = true;
type = 'depth';
%%

topic = 'medium_resolution/depth_registered/image';
topic_bag = select(bag, 'Topic', topic);

topic_frame_msg_cell = readMessages(topic_bag, frame); % take image message of frames of bag
disp('-Original');
topic_frame_msg = topic_frame_msg_cell{1} % extract from cell
img_original = readImage(topic_frame_msg);   % 480x640 single (32bit), with NaNs, [m] % !!!!!!!!!
img = img_original;

if rotation == true
    img = imrotate(img, 180);
end

if print == true
    figure(1)
    imshow(img);
end

if save_img == true
        filename = strcat('asl_window_', num2str(type), '_', num2str(frame), '_adj.png');
        img(isnan(img))=0;
        % convert from m to mm
        img = img * 1000;  
        % convert to uint16
        img = uint16(img);
        imwrite(img, filename, 'fmt', 'png');
end 

data_original = topic_frame_msg.Data;

% PNG image: uint16, [mm]