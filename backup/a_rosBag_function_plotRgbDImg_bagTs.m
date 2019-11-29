% Script to extract bag timestamps and plot images

clear; close all; clc;

filename = '/home/andreas/Documents/ros_recording/2019-10-15-17-48-49.bag';
bag = rosbag(filename);
bagInfo = rosbag('info',filename)

%% create cells from table of all topics and messages
topics_table = bag.AvailableTopics;
messages_table = bag.MessageList; % time with unit stamp (1.1.1970)

%% rgb image
rgb_source = 1
if rgb_source == 1
    rgb_bag_info = select(bag, 'Topic', '/medium_resolution/rgb_undistorted/camera_info');
    rgb_bag = select(bag, 'Topic', '/medium_resolution/rgb_undistorted/image');
elseif rgb_source ==2
    rgb_bag_info = select(bag, 'Topic', '/arduino_vi/rgb/camera_info');
    rgb_bag = select(bag, 'Topic', '/arduino_vi/rgb/image_raw');
end  
rgb_topics = rgb_bag.AvailableTopics;
rgb_messages_table = rgb_bag.MessageList;
rgb_messages = table2cell(rgb_messages_table);

rgb_firstMsg_cell = readMessages(rgb_bag,1);

img = readImage(rgb_firstMsg_cell{1,1});
img_rot = imrotate(img, 180);
figure()
imshow(img_rot);

% time stamp
rgb_time_cell = rgb_messages(:,1);
rgb_time = cell2mat(rgb_time_cell);
plot(diff(rgb_time))

%% depth image
depth_source = 1
if depth_source == 1
    depth_bag_info = select(bag, 'Topic', '/medium_resolution/depth_registered/camera_info')
    depth_bag = select(bag, 'Topic', '/medium_resolution/depth_registered/image')
elseif rgb_source ==2
    depth_bag_info = select(bag, 'Topic', '/arduino_vi/depth/camera_info')
    depth_bag = select(bag, 'Topic', '/arduino_vi/depth/image_raw')
end  

depth_topics = depth_bag.AvailableTopics;
depth_messages_table = depth_bag.MessageList;
depth_messages = table2cell(depth_messages_table);

depth_firstMsg_cell = readMessages(depth_bag,1);

img = readImage(depth_firstMsg_cell{1,1});
img_rot = imrotate(img, 180);
figure()
imshow(img_rot);

% time stamp
depth_time_cell = depth_messages(:,1);
depth_time = cell2mat(depth_time_cell);

%%
topic = '/medium_resolution/rgb_undistorted/image'

for frame = 400:1:400
    [img, time] = save_plot_img(bag, topic, frame, true);
end

%% Plot Time Synch of Data

figure()
plot(diff(rgb_time))
hold on
plot(diff(depth_time))
xlabel('Frames')
ylabel('Difference between timestamps of sequential frames [s]')
legend('/medium_resolution/rgb_undistorted/image','/medium_resolution/depth_registered/image', 'Interpreter', 'none')
title('Time synchronistation of measurements')


%%
function [img, time] = save_plot_img(bag, topic, frame, rotation)
    
    %% image
    % make subbag of specified topic only
    topic_bag = select(bag, 'Topic', topic);
    % take image message of frames of bag
    topic_msg_cell = readMessages(topic_bag, frame);
    
    % convert ros image format to array
    img = readImage(topic_msg_cell{1,1});
    if rotation == true
        img = imrotate(img, 180);
    end
    
    figure(1)
    imshow(img);
    filename = strcat('asl_window_', num2str(frame), '.jpg');
    imwrite(img, filename)
    
    %% time message
    % make table of all messages
    topic_messages_table = topic_bag.MessageList;
    % table -> cell
    topic_messages = table2cell(topic_messages_table);
    % take out timestep of the frame where image is read
    time = cell2mat(topic_messages(frame,1));
    
    return

end