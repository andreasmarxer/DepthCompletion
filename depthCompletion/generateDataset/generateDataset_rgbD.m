 % ALL IN ALL - only use this script for getting ordered message HEADER time stamps
% and the corresponding images

clear; close all; clc;

filename = '/media/andreas/SanDiskSSD/Semester_Thesis/ros_recording/2019-10-15-17-48-49_original.bag';
bag = rosbag(filename);
bagInfo = rosbag('info',filename);


%%
% ==========  settings ===================
rotation = true;
print = false;
save_img = false;
save_mat = false;
tolerance = 1e-3;
% ========================================

topic_1 = '/medium_resolution/rgb_undistorted/image';
type_1 = 'rgb';
topic_bag_1 = select(bag, 'Topic', topic_1);
topic_2 = 'medium_resolution/depth_registered/image';
type_2 = 'depth';
topic_bag_2 = select(bag, 'Topic', topic_2);

length_1 = topic_bag_1.NumMessages;
length_2 = topic_bag_2.NumMessages;
msg_time_vec_1 = zeros(length_1, 1);
msg_time_vec_2 = zeros(length_2, 1);

for frame = 1:1:length_1
msg_time_1 = return_msg_time(topic_bag_1, frame);
msg_time_vec_1(frame) = msg_time_1;
end

for frame = 1:1:length_2
msg_time_2 = return_msg_time(topic_bag_2, frame);
msg_time_vec_2(frame) = msg_time_2;
end
    
%% calculate correspondance indexes - use idx_long and idx_short
[idx_long, idx_short, idx_long_ext] = return_idx(msg_time_vec_1, msg_time_vec_2, tolerance);


%% Save and plot correspondant images

% for each correspondant
num_corr = size(idx_long,1);
time_1 = zeros(num_corr, 1);
time_2 = zeros(num_corr, 1);
num_corr = 3;
for label = 1:1:num_corr
    
    if topic_bag_1.NumMessages > topic_bag_2.NumMessages 
        img = plot_save_img(topic_bag_1, idx_long(label), type_1, label, rotation, print, save_img, save_mat, 'jpg');
        time_1(label) = return_msg_time(topic_bag_1, idx_long(label));
        img = plot_save_img(topic_bag_2, idx_short(label), type_2, label, rotation, print, save_img, save_mat, 'png');
        time_2(label) = return_msg_time(topic_bag_2, idx_short(label));
    elseif topic_bag_1.NumMessages < topic_bag_2.NumMessages 
        img = plot_save_img(topic_bag_1, idx_short(label), type_1, label, rotation, print, save_img, save_mat);
        time_1(label) = return_msg_time(topic_bag_1, idx_short(label));
        img = plot_save_img(topic_bag_2, idx_long(label), type_2, label, rotation, print, save_img, save_mat);
        time_2(label) = return_msg_time(topic_bag_1, idx_long(label));
    else % equal length - take case 1
        img = plot_save_img(topic_bag_1, idx_long(label), type_1, label, rotation, print, save_img, save_mat);
        time_1(label) = return_msg_time(topic_bag_1, idx_long(label));
        img = plot_save_img(topic_bag_2, idx_short(label), type_2, label, rotation, print, save_img, save_mat);
        time_2(label) = return_msg_time(topic_bag_2, idx_short(label));
    end
    
end



% %% Plots
% % Plot correspondant frames
% figure(1)
% scatter(1:size(idx_long_ext,1), idx_long_ext, 2,'filled')
% xlabel('rgb frames')
% ylabel('depth frames')
% title('Correspandance between rgb and depth camera message time stamps')
% 
% % Plot time diff of correspondant ordered times
% figure(2)
% plot(diff(time_1))
% set(gca, 'YScale', 'log')
% xlabel('Corresponding frames')
% ylabel('Framerate of corresponding frames [s]')
% legend('mean 0.4664,  variance 0.7750')
% title('Time synchronisation of RGB-D setup')


%% functions

% returns message content header time per frame
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
        print('Error in calculation, correspondent numers not equal !')
    end
    
    return
    
end


function img = plot_save_img(topic_bag, frame, label, type, rotation, print, save_img, save_mat, img_format)

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
            img(isnan(img))=0;
            % convert from m to mm
            img = img * 1000;  
            % convert to uint16
            img_uint16 = uint16(img);
            imwrite(img_uint16, filename, 'fmt', 'png');
        end
    end % of save_img
    
    if save_mat == true
        matfilename = strcat('asl_window_', num2str(type), '_original_', num2str(label), '.mat');
        save(matfilename, 'img');
    end
    
    return

end
