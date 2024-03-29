% script to generate dataset containing rgb and depth images
% as well as looked up pose at times corresponding with an
% tolerance of 1/1000s between rgb and depth time stamp

clear; close all; clc;

filename = '/media/andreas/SanDiskSSD/Semester_Thesis/ros_recording/original5_maplab_optimized_2019-11-28-13-18-04.bag';
bag = rosbag(filename);
bagInfo = rosbag('info',filename);

%%
% ==============================  settings ================================
rotation = true;    % need to be false for directly using for Open3d and matching with pose
                    % need to be true for generating images for depth adaptation
raster_correction = true; % median filter the depth image
kernel = 5;         % kernel size for median filter
print = false;      % print images as figures
save_img = false;   % save images (depth png, rgb jpeg)
save_mat = false;   % save image additionaly as array
debug = false;      % print msgs for debugging
tolerance_depth_rgb = 1/1000; %1ms tol in header time for correspondances
% =========================================================================

%% define topics to read from and respective length

topic_depth = 'medium_resolution/depth_registered/image';
type_depth = 'depth';
topic_bag_depth = select(bag, 'Topic', topic_depth);
length_depth = topic_bag_depth.NumMessages;
msg_time_vec_depth = zeros(length_depth, 1); % initialization

topic_rgb = '/medium_resolution/rgb_undistorted/image';
type_rgb = 'rgb';
topic_bag_rgb = select(bag, 'Topic', topic_rgb);
length_rgb = topic_bag_rgb.NumMessages;
msg_time_vec_rgb = zeros(length_rgb, 1);    % initialization

%% write header time of depth images and rgb images in vector

for frame = 1:1:length_depth
    msg_time_vec_depth(frame) = return_msg_time(topic_bag_depth, frame);
end

for frame = 1:1:length_rgb
    msg_time_vec_rgb(frame) = return_msg_time(topic_bag_rgb, frame);
end

%% calculate correspondance indexes of header times between depth and rgb

if length_rgb > length_depth
    [idx_rgb_depth, idx_depth_rgb] = return_idx(msg_time_vec_rgb, msg_time_vec_depth, tolerance_depth_rgb);
else
    [idx_depth_rgb, idx_rgb_depth] = return_idx(msg_time_vec_rgb, msg_time_vec_depth, tolerance_depth_rgb);
end

num_corr = size(idx_rgb_depth,1);


%% save images

% generate filename for pose
date_str = date;
temp = clock;
time_str = strcat('-', num2str(temp(4)), 'h-', num2str(temp(5)));
filename = strcat(date_str, time_str, 'rgbdPose_open3D');
% pose array
poses = zeros(4,4,num_corr);

% save RGB, DEPTH and look up and save POSE for correspondante depth time
for label = 2:num_corr
    label_write = label-1;
    
    if debug == true
        disp(label)
        disp(msg_time_vec_depth(idx_depth_rgb(label)))
    end
    
    % rgb part
    plot_save_img(topic_bag_rgb, idx_rgb_depth(label), label_write, type_rgb, rotation, print, save_img, save_mat, 'jpg', raster_correction, kernel);
    % depth part
    plot_save_img(topic_bag_depth, idx_depth_rgb(label), label_write, type_depth, rotation, print, save_img, save_mat, 'png', raster_correction, kernel);
    % pose lookup for calculated times (only available in bag until frame 372!)
    tf = getTransform(bag, 'mission', 'camera0', msg_time_vec_depth(idx_depth_rgb(label))); % T_mission_camera0
    % save text file for Open3D
    log_save_pose_lookedup(label, tf, filename)
    % save matlab array for gravity alignment
    poses(:,:,label) = return_pose_tfStamped_msg(tf);
end


%% functions

function msg_time = return_msg_time(topic_bag, frame)
% returns message content header time per frame
%
% input:    topic_bag:  topic selected from bag with select(bag, 'Topic', topic)
%           frame:      number of message to read from topic
%
% output:   msg_time:   header time in second (added up s and ns from bag header)

topic_frame_msg_cell = readMessages(topic_bag, frame); % take image message of frames from topic
topic_frame_msg = topic_frame_msg_cell{1}; % extract from cell

%% header message time 
sec = topic_frame_msg.Header.Stamp.Sec;
nsec = topic_frame_msg.Header.Stamp.Nsec;
time_s = sec + nsec/10^9;

msg_time = time_s; % seconds from 1.1.1970

return

end


function [idx_long, idx_short] = return_idx(msg_time_vec_1, msg_time_vec_2, tolerance)
% returns indexes of correspondances long and short image header time vector
%
% input:    msg_time_vec_1: header time vector in [s] of first image
%           msg_time_vec_2: header time vector in [s] of second image
%           tolerance: time in [s] in which interval we consider as correspondante
%
% output:   index of time vectors which correspond considering tolerance

% define long and short vector
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

% initialize index vectors
idx_long = zeros(size(long_vec,1),1);
idx_short = zeros(size(short_vec,1),1);

idx = 1;

% iterate through short vector
for k_short=1:1:size(short_vec,1)
    % calculate distance to whole long vector from current value of short
    dist = abs(short_vec(k_short)-long_vec);
    % correspondante if distance is smaller than tolerance
    corr_long = find(dist < tolerance);
        
    if ~isempty(corr_long)
        % when size longer than 1, take minimum of dist(corr_long)
        if size(corr_long,1) > 1
            disp('Multiple correspondant found')
            corr_long = find (dist == min(dist(corr_long)));
        end
        % save correspondance indexes in output vector
        idx_long(idx) = corr_long;
        idx_short(idx) = k_short;
        % increment correspondance count
        idx = idx + 1;
    else
        disp('No correspondant found')
    end
   
end

% cut the vectors after last correspondence
if nnz(idx_long) == nnz (idx_short)
idx_long = idx_long(1:nnz(idx_long));
idx_short = idx_short(1:nnz(idx_short));
else
print('Error in calculation, correspondent numbers not equal !')
end

return

end


function plot_save_img(topic_bag, frame, label, type, rotation, print, save_img, save_mat, img_format, raster_correction, kernel)
% plots and saves img according to specifications defined in input
%
% input:    topic_bag:  topic of bag
%           frame:      message number in topic
%           label:      correspondance number to numerate images
%           type:      'rgb' or 'depth'
%           rotation:   boolean, false=overhead and true=normal
%           print:      boolean, display figures during runtime
%           save_img:   boolean, save image in img_format
%           save_mat:   boolean, save image as array
%           img_format: 'jpg' for rgb, 'png' for depth
%           raster_correction:  boolean, if true median filter is applied on depth
%           kernel:     kernel size for raster correction
%
% output:   index of time vectors which correspond considering tolerance

if nargin < 9
        img_format = 'jpg';
        raster_correction = false;
end

topic_frame_msg_cell = readMessages(topic_bag, frame); % take image message of frames of bag
topic_frame_msg = topic_frame_msg_cell{1}; % extract from cell
% convert ros image format to array
img = readImage(topic_frame_msg);
    
if rotation == true
    img = imrotate(img, 180);
end

% median filter the depth image
if raster_correction == true && type(1) == 'd'
    kernel_half = (kernel-1)/2;
    depth_img_woraster = img;
    for x = kernel_half+1:1:size(img,2)-kernel_half
        for y = kernel_half+1:1:size(img,1)-kernel_half
            % only apply filter if value is not available !!!
            if ~(depth_img_woraster(y,x) > 0)
                depth_img_woraster(y,x) = kernelMedian(kernel,x,y,img);
            end
        end
    end
end

if print == true
    figure(1)
    imshow(img);
end

if save_img == true
    % rgb images
    if all(img_format == 'jpg')
        filename = strcat('asl_window_', num2str(type), '_', num2str(label), '.jpg');
        imwrite(img, filename);
    % depth images
    elseif all(img_format == 'png')
        filename_depth = strcat('asl_window_', num2str(type), '_', num2str(label), '.png');
        % convert Nans to zeros
        img(isnan(img))=0;
        % convert from m to mm
        img = img * 1000;
        % convert to uint16
        img = uint16(img);
        imwrite(img, filename_depth, 'fmt', 'png'); 
        
        if raster_correction == true 
            filename_depth_median = strcat('asl_window_', num2str(type), '_median', num2str(kernel), '_', num2str(label), '.png');
            % convert Nans to zeros
            depth_img_woraster(isnan(img))=0;
            % convert from m to mm
            depth_img_woraster = depth_img_woraster * 1000;
            % convert to uint16
            depth_img_woraster = uint16(depth_img_woraster);
            imwrite(depth_img_woraster, filename_depth_median, 'fmt', 'png');
        end
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
% saves txt file with looked up poses for Open3D rgbd_integration.py
%
% input:    label: number matching with images numbering
%           tf: transformation
%           filename: name where to save txt file
%
% output:   none, creates and saves file in current folder

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


function pose = return_pose_tfStamped_msg(tf)
% transforms tf pose format (quaternion) to transformation matrix
%
% input:    tf:     transformation
%
% output:   pose:   transformation matrix (4x4)

% rotation
q_x = tf.Transform.Rotation.X;
q_y = tf.Transform.Rotation.Y;
q_z = tf.Transform.Rotation.Z;
q_w = tf.Transform.Rotation.W;
quat = [q_w, q_x, q_y, q_z];
R_cw = quat2rotm(quat); % quaternion to rotation matrix

% translation
t_x = tf.Transform.Translation.X;
t_y = tf.Transform.Translation.Y;
t_z = tf.Transform.Translation.Z;
t_1 = 1;

t_cw = [t_x; t_y; t_z];
T_cw = [t_cw; 1];

% initialize pose
pose = zeros(4,4);

% output
pose(1:3,1:3) = R_cw;
pose(1:4,4) = T_cw;


return
    
end
