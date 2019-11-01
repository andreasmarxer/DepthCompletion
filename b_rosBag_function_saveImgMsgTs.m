% 1. Save numbered images to folder where this script is located.
% 2. Generate msg_time_vec which contains all message timestamps in order as
%    received the messages

clear; close all; clc;

filename = '/home/andreas/Documents/ros_recording/2019-10-15-17-48-49.bag';
bag = rosbag(filename);
bagInfo = rosbag('info',filename)


%%
% ==========  settings ===================
debug = true;
rgb = true;
% ----------------------------------------
rotation = true;
print = true;
save = false;
% ========================================
if  debug == true
    topic = '/medium_resolution/rgb_undistorted/image'
    length = 10;
    
elseif rgb == true
    topic = '/medium_resolution/rgb_undistorted/image'
    length = 671;
    
elseif rgb == false
    topic = '/medium_resolution/depth_registered/image'
    length = 623;
end

msg_time_vec = zeros(length,1);
    
for frame = 1:1:length
    % [img, msg_time] = save_plot_img(bag, topic, frame, rotation=t/f, print=t/f, save=t/f)
    [img, msg_time] = save_plot_img(bag, topic, frame, rotation, print, save);
    msg_time_vec(frame) = msg_time;
    clear msg_time;
end



%%
function [img, msg_time] = save_plot_img(bag, topic, frame, rotation, print, save)
    
    %% image
    % make subbag of specified topic only
    topic_bag = select(bag, 'Topic', topic);
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
    elseif save == true
        filename = strcat('asl_window_', num2str(frame), '.jpg');
        imwrite(img, filename)
    end
    
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