clear; close all; clc;

filename = '/home/andreas/Documents/ros_recording/2019-10-15-17-48-49.bag';
bag = rosbag(filename);
bagInfo = rosbag('info',filename);

frame = 302;

%%
topics_table = bag.AvailableTopics;
messages_table = bag.MessageList; % time with unit stamp (1.1.1970)

%%
topic_1 = '/tf';
topic_bag_1 = select(bag, 'Topic', topic_1);
topic_frame_msg_cell_1 = readMessages(topic_bag_1, frame); % take image message of frames of bag
topic_frame_msg = topic_frame_msg_cell_1{1}; % extract from cel


showdetails(topic_frame_msg)
%%
if topic_frame_msg.Transforms.Header.FrameId == "mission"
    msg_time = return_msg_time_tf2_msg(topic_frame_msg, frame);
    pose = return_pose_tf2_msg(topic_frame_msg, frame);
else
    disp('Actual frame doesnt contain correct pose msg')
end

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
    quat = [q_x, q_y, q_z, q_w];

    R = quat2rotm(quat);
    % return pose
    pose = zeros(4,4);
    pose(1:3,1:3) = R;
    pose(1:4,4) = T;
    
    return
    
end