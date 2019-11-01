clear; close all; clc;

filename = '/home/andreas/Documents/ros_recording/2019-10-15-17-48-49.bag';
bag = rosbag(filename);
bagInfo = rosbag('info',filename);

%%
topics_table = bag.AvailableTopics;
messages_table = bag.MessageList; % time with unit stamp (1.1.1970)

%%
topic_1 = '/tf';
topic_bag_1 = select(bag, 'Topic', topic_1);

topic_2 = 'medium_resolution/depth_registered/image';
topic_bag_2 = select(bag, 'Topic', topic_2);

topic_3 = '/medium_resolution/rgb_undistorted/image';
topic_bag_3 = select(bag, 'Topic', topic_3);

%%
length_1 = topic_bag_1.NumMessages;
length_2 = topic_bag_2.NumMessages;
length_3 = topic_bag_3.NumMessages;
msg_time_vec_1 = NaN(length_1, 1);
msg_time_vec_2 = NaN(length_2, 1);
msg_time_vec_3 = NaN(length_3, 1);

for frame = 1:1:length_1
    topic_frame_msg_cell_1 = readMessages(topic_bag_1, frame); % take image message of frames of bag
    topic_frame_msg_1 = topic_frame_msg_cell_1{1}; % extract from cell
    if topic_frame_msg_1.Transforms.Header.FrameId == "mission"
        msg_time_1 = return_msg_time_tf2_msg(topic_frame_msg_1);
        msg_time_vec_1(frame) = msg_time_1;
    end
end

for frame = 1:1:length_2
    msg_time_2 = return_msg_time(topic_bag_2, frame);
    msg_time_vec_2(frame) = msg_time_2;
    
end

for frame = 1:1:length_3
    msg_time_3 = return_msg_time(topic_bag_3, frame);
    msg_time_vec_3(frame) = msg_time_3;
    
end

%% calculate correspondance indexes between depth image and pose
tolerance = 1/50; %20ms
[idx_pose_depth, idx_depth_pose, ~] = return_idx(msg_time_vec_1, msg_time_vec_2, tolerance);


%% calculate correspondance indexes between depth image and rgb images
% all previously used depth images have guaranteed a correspondance rgb due
% to the fact that this was checked previously --> just need the idx
tolerance = 1/1000; %1ms
[idx_rgb_depth, idx_depth_rgb, ~] = return_idx(msg_time_vec_3, msg_time_vec_2, tolerance);


%%
num_corr_pose = size(idx_pose_depth,1);
idx_rgb_pose = NaN(size(idx_pose_depth));
all_label = 0;

for label = 1:1:num_corr_pose
    
    if nnz(idx_depth_rgb == idx_depth_pose(label)) ~= 0
        all_label = all_label + 1;
        res = find(idx_depth_rgb == idx_depth_pose(label));
        idx_rgb_pose(all_label) = idx_rgb_depth(res);
        idx_pose_depth(all_label) = idx_pose_depth(label);
        idx_depth_pose(all_label) = idx_depth_pose(label);
    else
        continue
    end
end

idx_rgb_all3 = idx_rgb_pose(1:all_label);
idx_pose_all3 = idx_pose_depth(1:all_label);
idx_depth_all3 = idx_depth_pose(1:all_label);
    

%% functions

% returns message content header time per frame
function msg_time = return_msg_time_tf2_msg(topic_frame_msg)
    %% time message
    sec = topic_frame_msg.Transforms.Header.Stamp.Sec;
    nsec = topic_frame_msg.Transforms.Header.Stamp.Nsec;
    time_ns = sec*10^9 + nsec;
    time_s = sec + nsec/10^9;
    msg_time = time_s; % seconds from 1.1.1970
    % print date and time
    datetime(time_ns,'ConvertFrom','epochtime','TicksPerSecond',1e9,'Format','dd-MMM-yyyy HH:mm:ss.SSSSSSSSS');
    
    return

end


function msg_time = return_msg_time(topic_bag, frame)
    topic_frame_msg_cell = readMessages(topic_bag, frame); % take image message of frames of bag
    topic_frame_msg = topic_frame_msg_cell{1}; % extract from cell
    
    %% time message
    sec = topic_frame_msg.Header.Stamp.Sec;
    nsec = topic_frame_msg.Header.Stamp.Nsec;
    time_ns = sec*10^9 + nsec;
    time_s = sec + nsec/10^9;
    msg_time = time_s; % seconds from 1.1.1970
    % print date and time
    datetime(time_ns,'ConvertFrom','epochtime','TicksPerSecond',1e9,'Format','dd-MMM-yyyy HH:mm:ss.SSSSSSSSS');
    
    return

end


function [idx_long, idx_short, idx_long_ext] = return_idx(msg_time_vec_1, msg_time_vec_2, tolerance)
    
    if size(msg_time_vec_1,1) < size(msg_time_vec_2,1)
        short_vec = msg_time_vec_1;
        long_vec = msg_time_vec_2;
    elseif size(msg_time_vec_1,1) > size(msg_time_vec_2,1)
        short_vec = msg_time_vec_2;
        long_vec = msg_time_vec_1;
    else %equal length - take case 1
        short_vec = msg_time_vec_1;
        long_vec = msg_time_vec_2;
    end

    idx_long_ext = zeros(size(long_vec,1),1);
    idx_long = zeros(size(long_vec,1),1);
    idx_short = zeros(size(short_vec,1),1);
    
    idx = 1;
    % iterate through short vector
    for k_short=1:1:size(short_vec,1)
        dist = abs(short_vec(k_short)-long_vec);
        corr_long = find(dist < tolerance);
        % which long_vec idx corresponds to short_vec(k_short) ?
        
        if ~isempty(corr_long)
            %%
            % when size longer than 1, take minimum of dist(corr_long)
            %%
            idx_long(idx) = corr_long;
            idx_short(idx) = k_short;
            idx = idx + 1;
            % idx of long vector contains number of correspondent short vec idx
            idx_long_ext(corr_long) = k_short;
        end

    end
    
    if nnz(idx_long) == nnz (idx_short)
        idx_long = idx_long(1:nnz(idx_long));
        idx_short = idx_short(1:nnz(idx_short));
    else
        print('Error in calculation, correspondent numbers not equal !')
    end
    
    return
    
end


function pose = return_pose_tf2_msg(topic_frame_msg, frame)
    % translation
    t_x = topic_frame_msg.Transforms.Transform.Translation.X;
    t_y = topic_frame_msg.Transforms.Transform.Translation.Y;
    t_z = topic_frame_msg.Transforms.Transform.Translation.Z;
    t_1 = 1;
    T = [t_x; t_y; t_z; t_1];
    % rotation
    q_x = topic_frame_msg.Transforms.Transform.Rotation.X;
    q_y = topic_frame_msg.Transforms.Transform.Rotation.Y;
    q_z = topic_frame_msg.Transforms.Transform.Rotation.Z;
    q_w = topic_frame_msg.Transforms.Transform.Rotation.W;
    quat = [q_w, q_x, q_y, q_z];

    R = quat2rotm(quat);
    % return pose
    pose = zeros(4,4);
    pose(1:3,1:3) = R;
    pose(1:4,4) = T;
    
    return
    
end