clear; close all; clc;

filename = '/media/andreas/SanDiskSSD/Semester_Thesis/ros_recording/2019-10-15-17-48-49_original.bag';
%filename = '/media/andreas/SanDiskSSD/Semester_Thesis/ros_recording/z_calibration_etc/calib.bag';
bag = rosbag(filename);
bagInfo = rosbag('info',filename);

%%
% ==========  settings ===================
rotation = false;
print = false;
save_img = false;
save_pose = true;
save_mat = false;
tolerance_depth_pose = 1/50; %20ms
tolerance_depth_rgb = 1/1000; %1ms
% ========================================

%%

topic_depth = 'medium_resolution/depth_registered/image';
type_depth = 'depth';
topic_bag_depth = select(bag, 'Topic', topic_depth);
length_depth = topic_bag_depth.NumMessages;
msg_time_vec_depth = zeros(length_depth, 1);

topic_rgb = '/medium_resolution/rgb_undistorted/image';
type_rgb = 'rgb';
topic_bag_rgb = select(bag, 'Topic', topic_rgb);
length_rgb = topic_bag_rgb.NumMessages;
msg_time_vec_rgb = zeros(length_rgb, 1);
%%

for frame = 1:1:length_depth
    msg_time_depth = return_msg_time(topic_bag_depth, frame);
    msg_time_vec_depth(frame) = msg_time_depth;
end

for frame = 1:1:length_rgb
    msg_time_rgb = return_msg_time(topic_bag_rgb, frame);
    msg_time_vec_rgb(frame) = msg_time_rgb;
end

%% calculate correspondance indexes between depth image and rgb images
[idx_rgb_depth, idx_depth_rgb, ~] = return_idx(msg_time_vec_rgb, msg_time_vec_depth, tolerance_depth_rgb);

num_corr = size(idx_rgb_depth,1);
    
%% Debugging

for i = 1:num_corr
%     disp(msg_time_vec_3(idx_rgb_depth(i)))
%     disp(msg_time_vec_2(idx_depth_rgb(i)))
%     disp('---------')
      diff = msg_time_vec_rgb(idx_rgb_depth(i))-msg_time_vec_depth(idx_depth_rgb(i));
      test(i) = diff > tolerance_depth_rgb;
end

nnz(test)

%% save images
for label = 1:num_corr
%for label = 100:1:199
    %label_write = label-99;
    label_write = label;
    % rgb
    plot_save_img(topic_bag_rgb, idx_rgb_depth(label), type_rgb, label_write, rotation, print, save_img, save_mat, 'jpg');
    % depth
    plot_save_img(topic_bag_depth, idx_depth_rgb(label), type_depth, label_write, rotation, print, save_img, save_mat, 'png');
end


%% LAUNCH simulation time
%% start this section
%% PLAY ROS BAG after
%% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!


%% lookup and save pose
clear tf; clear tf_tree;

% if save_pose == true
%     rosinit; 
%     tf_tree = rostf; % is an ongoing variable
%     tf_tree.BufferTime = Inf; % standard is 10, but not enough fails at 365
%     
%     while size(tf_tree.AvailableFrames,1) == 0
%         disp('waiting')
%     end
    date_str = date;
    temp = clock;
    time_str = strcat('-', num2str(temp(4)), 'h-', num2str(temp(5)));
    filename = strcat(date_str, time_str, 'rgbdPose_open3D');
    %for c = 1:num_corr
    for c = 1:num_corr
        % get msg_time_vec_1 and idx_pose_all3 from generateDataset_rgbDP
        %tf = getTransform(tf_tree, 'mission', 'imu', msg_time_vec_pose_all(idx_pose_all(c)), "Timeout", 20);
        % --> should be the same as without lookup !!!
        
        %ORIGINAL LOOKUP
        %tf = getTransform(tf_tree, 'mission', 'imu', msg_time_vec_depth(idx_depth_rgb(c)), 'Timeout', Inf);
        
        tf = getTransform(bag, 'mission', 'camera0', msg_time_vec_depth(idx_depth_rgb(c)));
        % tragetframe first, sourceframe second !!!
%         time_part_s = tf.Header.Stamp.Sec;
%         time_part_ns = tf.Header.Stamp.Nsec;
%         time_s = time_part_s + time_part_ns/10^9;
% %         
        disp(c)
        disp(msg_time_vec_depth(idx_depth_rgb(c))) % ORIGINAL
        %disp(msg_time_vec_pose_all(idx_pose_all(c)))
%         disp('Found tf - header time')
%         disp(time_s)
%         disp('=======================')
        log_save_pose_lookedup(c, tf, filename)
    end
%end
%%

%log_save_pose_lookedup(1, test, 'test')

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
            if size(corr_long,1) > 1
                disp('Multiple correspondant found')
            end
            %%
            idx_long(idx) = corr_long;
            idx_short(idx) = k_short;
            idx = idx + 1;
            % idx of long vector contains number of correspondent short vec idx
            idx_long_ext(corr_long) = k_short;
        else
            disp('No correspondant found')
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


function plot_save_img(topic_bag, frame, label, type, rotation, print, save_img, save_mat, img_format)

    topic_frame_msg_cell = readMessages(topic_bag, frame); % take image message of frames of bag
    topic_frame_msg = topic_frame_msg_cell{1}; % extract from cell
    % convert ros image format to array
    img = readImage(topic_frame_msg);
    if rotation == true
        img = imrotate(img, 180);
    end
    
    if print == true
        figure(1)
        imshow(img);
    end
    
    if save_img == true
        if nargin < 9
            img_format == 'jpg';
        end
        % rgb images
        if img_format == 'jpg'
            filename = strcat('asl_window_', num2str(type), '_', num2str(label), '.jpg');
            imwrite(img, filename);
        % depth images
        elseif img_format == 'png'
            filename = strcat('asl_window_', num2str(type), '_', num2str(label), '.png');
            % convert Nans to zeros
            img(isnan(img))=0;
            % convert from m to mm
            img = img * 1000;  
            % convert to uint16
            img_uint16 = uint16(img);
            imwrite(img_uint16, filename, 'fmt', 'png');
        end
    end % of save_img
    
    % save data also as mat file
    if save_mat == true
        matfilename = strcat('asl_window_', num2str(type), '_original_', num2str(label), '.mat');
        save(matfilename, 'img');
    end
    
    return

end


function log_save_pose_lookedup(label, tf, filename)
        fid = fopen(filename,'a');
        fprintf( fid, '%d\t%d\t%d\n', label-1, label-1, label );
        
        invert = false;
        m = return_pose_tfStamped_msg(tf, invert);
        fprintf( fid, '%.10f\t%.10f\t%.10f\t%.10f\n', m(1,1), m(1,2), m(1,3), m(1,4) );
        fprintf( fid, '%.10f\t%.10f\t%.10f\t%.10f\n', m(2,1), m(2,2), m(2,3), m(2,4) );
        fprintf( fid, '%.10f\t%.10f\t%.10f\t%.10f\n', m(3,1), m(3,2), m(3,3), m(3,4) );
        fprintf( fid, '%.10f\t%.10f\t%.10f\t%.10f\n', m(4,1), m(4,2), m(4,3), m(4,4) );
        fclose(fid);
end


function pose = return_pose_tfStamped_msg(tf, invert)
    % rotation
    q_x = tf.Transform.Rotation.X;
    q_y = tf.Transform.Rotation.Y;
    q_z = tf.Transform.Rotation.Z;
    q_w = tf.Transform.Rotation.W;
    quat = [q_w, q_x, q_y, q_z];
    
    R_cw = quat2rotm(quat);
    if invert == true
        R_wc = R_cw';
    end
    
    % translation
    t_x = tf.Transform.Translation.X;
    t_y = tf.Transform.Translation.Y;
    t_z = tf.Transform.Translation.Z;
    t_1 = 1;
    
    t_cw = [t_x; t_y; t_z];
    T_cw = [t_cw; 1];
    if invert == true
        t_wc = -R_wc*t_cw;
        T_wc = [t_wc; 1];
    end
    
    % return pose
    pose = zeros(4,4);
     
   if invert == true
        pose(1:3,1:3) = R_wc;
        pose(1:4,4) = T_wc;
   else
        pose(1:3,1:3) = R_cw;
        pose(1:4,4) = T_cw;
   end
    
    return
    
end


% old functions
function log_save_pose(topic_bag, label, frame)
        fid = fopen('rgbdPose_open3D.txt','a');
        fprintf( fid, '%d\t%d\t%d\n', label-1, label-1, label );
        m = return_pose_tf2_msg(topic_bag, frame);
        fprintf( fid, '%.10f\t%.10f\t%.10f\t%.10f\n', m(1,1), m(1,2), m(1,3), m(1,4) );
        fprintf( fid, '%.10f\t%.10f\t%.10f\t%.10f\n', m(2,1), m(2,2), m(2,3), m(2,4) );
        fprintf( fid, '%.10f\t%.10f\t%.10f\t%.10f\n', m(3,1), m(3,2), m(3,3), m(3,4) );
        fprintf( fid, '%.10f\t%.10f\t%.10f\t%.10f\n', m(4,1), m(4,2), m(4,3), m(4,4) );
        fclose(fid);
end

function pose = return_pose_tf2_msg(topic_bag, frame)
    % get frame message
    topic_frame_msg_cell = readMessages(topic_bag, frame); % take image message of frames of bag
    topic_frame_msg = topic_frame_msg_cell{1}; % extract from cell
    
    % rotation
    q_x = topic_frame_msg.Transforms.Transform.Rotation.X;
    q_y = topic_frame_msg.Transforms.Transform.Rotation.Y;
    q_z = topic_frame_msg.Transforms.Transform.Rotation.Z;
    q_w = topic_frame_msg.Transforms.Transform.Rotation.W;
    quat = [q_w, q_x, q_y, q_z];

    R_cw = quat2rotm(quat);
    R_wc = R_cw';
    
    % translation
    t_x = topic_frame_msg.Transforms.Transform.Translation.X;
    t_y = topic_frame_msg.Transforms.Transform.Translation.Y;
    t_z = topic_frame_msg.Transforms.Transform.Translation.Z;
    t_1 = 1;
    
    t_cw = [t_x; t_y; t_z];
    t_wc = -R_wc*t_cw;
    T_wc = [t_wc; 1];
    
    
    % return pose
    pose = zeros(4,4);
    pose(1:3,1:3) = R_wc;
    pose(1:4,4) = T_wc;
    
    return
    
end

function log_save_info(label, c)
        fid = fopen('rgbdInfo_open3D.txt','a');
        fprintf( fid, '%d\t%d\t%d\n', label-1, label-1, label );
        fprintf( fid, '%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\n', c(1,1), c(1,2), c(1,3), c(1,4), c(1,5), c(1,6) );
        fprintf( fid, '%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\n', c(2,1), c(2,2), c(2,3), c(2,4), c(2,5), c(2,6) );
        fprintf( fid, '%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\n', c(3,1), c(3,2), c(3,3), c(3,4), c(3,5), c(3,6) );
        fprintf( fid, '%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\n', c(4,1), c(4,2), c(4,3), c(4,4), c(4,5), c(4,6) );
        fprintf( fid, '%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\n', c(5,1), c(5,2), c(5,3), c(5,4), c(5,5), c(5,6) );
        fprintf( fid, '%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\n', c(6,1), c(6,2), c(6,3), c(6,4), c(6,5), c(6,6) );
end