clear; close all; clc;

filename = '2019-10-15-17-48-49.bag';
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
depth_source = 2
if depth_source == 1
    depth_bag_info = select(bag, 'Topic', '/medium_resolution/depth_registered/camera_info')
    depth_bag = select(bag, 'Topic', '/medium_resolution/depth_registered/image')
elseif depth_source ==2
    depth_bag_info = select(bag, 'Topic', '/arduino_vi/depth/camera_info')
    depth_bag = select(bag, 'Topic', '/medium_resolution/depth_registered/image')
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


%% Plot Time Synch of Data

figure()
plot(diff(rgb_time))
hold on
plot(diff(depth_time))
xlabel('Frames')
ylabel('Difference between timestamps of sequential frames [s]')
legend('/medium_resolution/rgb_undistorted/image','/medium_resolution/depth_registered/image', 'Interpreter', 'none')
title('Time synchronistation of measurements')

