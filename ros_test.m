clear; clc; close all;

filename = '/home/andreas/Documents/ros_recording/2019-10-15-17-48-49_original.bag';
bag = rosbag(filename);
bagInfo = rosbag('info',filename);

%%

topic = '/depth_adapted/image';
%topic = '/binary_predictions/image';
frame = 1;

topic_bag = select(bag, 'Topic', topic);
topic_frame_msg_cell = readMessages(topic_bag, frame); % take image message of frames of bag
topic_frame_msg = topic_frame_msg_cell{1}; % extract from cell
img = readImage(topic_frame_msg);
hold on
imshow(img);


sec = topic_frame_msg.Header.Stamp.Sec;
nsec = topic_frame_msg.Header.Stamp.Nsec;
time_ns = sec*10^9 + nsec;
time_s = sec + nsec/10^9;

%%

topic = '/tf';
%topic = '/tf_static';
frame = 2;

topic_bag = select(bag, 'Topic', topic);
topic_frame_msg_cell = readMessages(topic_bag, frame); % take image message of frames of bag
topic_frame_msg = topic_frame_msg_cell{1}; % extract from cell

showdetails(topic_frame_msg)

sec = topic_frame_msg.Transforms.Header.Stamp.Sec;
nsec = topic_frame_msg.Transforms.Header.Stamp.Nsec;
time_ns = sec*10^9 + nsec;
time_s = sec + nsec/10^9;