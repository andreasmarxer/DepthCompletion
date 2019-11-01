clear; clc;
rosshutdown;

try
   %% initialization
    rosinit;

    %% definitions
    
    % initial publisher for testing
%     pub = rospublisher('/medium_resolution/depth_registered/image', 'sensor_msgs/Image');
%     pause(1);
    
    % publisher global
    pub_global = rospublisher('/depth_adapted/image', 'sensor_msgs/Image');
    setGlobalPub(pub_global);
    pause(1);

    % subscriber
    sub_depth = rossubscriber('/medium_resolution/depth_registered/image', 'sensor_msgs/Image', @sensor_msgsImageCallback);
    pause(1);

    %% set time correspondance vector for comparing in sensor_msgsImageCallback
    setGlobalTimeCorresp;

    %% testing
%     msg_test = rosmessage('sensor_msgs/Image');
%     send(pub, msg_test);
    
catch exception
    rosshutdown;
    throw(exception);
end

%% functions

function sensor_msgsImageCallback(~, msg_in)
    disp("RECEIVED depth_registered/image message, Doing Callback !"); 
    
    % convert header time stamp to seconds
    sec = msg_in.Header.Stamp.Sec;
    nsec = msg_in.Header.Stamp.Nsec;
    time_s = sec + nsec/10^9;
    disp(strcat('Header time stamp: ', num2str(time_s)));
    
    % get time vector of all correspondant images
    time_corresp = getGlobalTimeCorresp;
    
    % if the current message is one of the correspondant images
    % only the correspondant images have predictions and are adjusted !!!
    if nnz(time_s == time_corresp) == 1
        disp('=== Correspondant found !!!');
        
        % get global publisher
        pub_global = getGlobalPub;
        % define message type
        msg_out = rosmessage('sensor_msgs/Image');
        
    
        %% copy properties
        % copy header with time stamp
        msg_out.Header = msg_in.Header;
%         msg_out.Header.Seq = msg_in.Header.Seq;
%         msg_out.Header.Stamp.Sec = msg_in.Header.Stamp.Sec;
%         msg_out.Header.Stamp.Nsec = msg_in.Header.Stamp.Nsec;
%         msg_out.Header.FrameId = msg_in.Header.FrameId;
        % copy width and height
        msg_out.Height = msg_in.Height;
        msg_out.Width = msg_in.Width;
        % copy encoding type 32FC1 (single floating point)
        msg_out.Encoding = msg_in.Encoding;
        % copy other staff
        msg_out.Step = msg_in.Step;
        msg_out.IsBigendian = msg_in.IsBigendian;     
        
        %% get adjusted depth image and process for writting to message
        label = 2;
        rotation_back = true;
        
        path = '/home/andreas/Documents/ASL_window_dataset/depth_images_adj/';
        filename = strcat(path, 'asl_window_', num2str(label), '_depth_adj.png');

        %%
        img = imread(filename); % PNG image: uint16, [mm]

        img = single(img); % backtransform to single
        img(img < 0.0001) = NaN; % backtransform zeros to NaN
        img = img/1000; % backtransform to [m]

        if rotation_back == true
            img = imrotate(img, 180);
        end

        %% write depth image to data
        writeImage(msg_out, img);

        
        %% publish message
        send(pub_global, msg_out);
    
    end

end

%% global publisher

function setGlobalPub(pub)
    global x
    x = pub;
end


function r = getGlobalPub
    global x
    r = x;
end

%% global timevector of correspondant times

function setGlobalTimeCorresp
    global y
    % ONLY change this 1 line here !!! -----------------------------------
    time_struct = load('/home/andreas/Documents/MATLAB/time_corresp_2to9.mat');
    % ONLY change this 1 line here !!! -----------------------------------
    y = time_struct.time_corresp;
end

function time_vec = getGlobalTimeCorresp
    global y
    time_vec = y;
end

