clear; clc; close all;
%%=========================================================================
filename = '/home/andreas/test_fix.bag';
bag = rosbag(filename);
bagInfo = rosbag('info',filename);

topic = '/depth_adapted/image';
frame = 1;

topic_bag = select(bag, 'Topic', topic);
topic_frame_msg_cell = readMessages(topic_bag, frame); % take image message of frames of bag
topic_frame_msg = topic_frame_msg_cell{1}; % extract from cell

sec = topic_frame_msg.Header.Stamp.Sec;
nsec = topic_frame_msg.Header.Stamp.Nsec;
time_s = sec + nsec/10^9;