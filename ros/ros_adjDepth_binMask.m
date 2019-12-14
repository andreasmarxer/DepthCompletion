clear; clc;
rosshutdown;

% set print statements on/off with debug mode
debug = false;


try
   %% initialization
    rosinit;

    %% definitions
    setDebug(debug)
    
    % publisher global
    pub_global_adjDepth = rospublisher('/depth_adapted/image', 'sensor_msgs/Image');
    setGlobalPub(pub_global_adjDepth);
    pause(1);
    
    pub_global_adjDepth_info = rospublisher('/depth_adapted/camera_info', 'sensor_msgs/CameraInfo');
    setGlobalPubInfo(pub_global_adjDepth_info);
    pause(1);
    
    pub_global_predictions = rospublisher('/binary_predictions/image', 'sensor_msgs/Image');
    setGlobalPubBoudingBoxes(pub_global_predictions);
    pause(1);

    % subscriber
    sub_depth = rossubscriber('/medium_resolution/depth_registered/image', 'sensor_msgs/Image', @sensor_msgsImageCallback);
    pause(1);

    sub_depth_info = rossubscriber('/medium_resolution/depth_registered/camera_info', 'sensor_msgs/CameraInfo', @sensor_msgsInfoCallback);
    pause(1);

    % load global variable time correspondance vector
    setGlobalTimeCorresp;
    % load global struct with bouding box predictions
    setGlobalBoundingBoxes;
    
catch exception
    rosshutdown;
    throw(exception);
end


%% functions

function sensor_msgsImageCallback(~, msg_in)
    if getDebug
        disp("RECEIVED depth_registered/image message, Doing Callback !");
    end
    
    % convert header time stamp to seconds
    sec = msg_in.Header.Stamp.Sec;
    nsec = msg_in.Header.Stamp.Nsec;
    time_s = sec + nsec/10^9;
    if getDebug
        disp(strcat('Header time stamp: ', num2str(time_s)));
    end
    
    % get time vector of all correspondant images
    %time_corresp = getGlobalTimeCorresp;
    % get struct with bouding boxes
    struct_bb_pred = getGlobalBoundingBoxes;
    
    cell_bb_pred = struct2cell(struct_bb_pred);
    cell_time_stamp = cell_bb_pred(1,1,:);
    array_time_stamp = cell2mat(cell_time_stamp(:));
        
    % if the current message is one of the correspondant images
    % only the correspondant images have predictions and are adjusted !!!
    if nnz(time_s == array_time_stamp) == 1
        label = find(time_s == array_time_stamp);
      
        asl_train_labels = [1,16,39,90,128,178,199,221,264,269,283,289,307,337,350,355,361,363,365,368];

        if ismember(label, asl_train_labels)
            if getDebug
                disp('-- Training image, return! ');
            end
            return;
        else
            
            if getDebug
                disp(strcat('++ Correspondant found ! - Label: ', num2str(label)));
            end

            % get global publisher
            pub_global_adjDepth = getGlobalPub;
            % define message type
            msg_out = rosmessage('sensor_msgs/Image');

            %% copy properties
            % copy header with time stamp
            msg_out.Header = msg_in.Header;
            % copy width and height
            msg_out.Height = msg_in.Height;
            msg_out.Width = msg_in.Width;
            % copy encoding type 32FC1 (single floating point)
            msg_out.Encoding = msg_in.Encoding;
            % copy other staff
            msg_out.Step = msg_in.Step;
            msg_out.IsBigendian = msg_in.IsBigendian;     
        
            %% get adjusted depth image and process for writting to message
            rotation_back = true;

            path = '/home/andreas/Documents/ASL_window_dataset/depth_images_adj/e/'; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            filename = strcat(path, 'asl_window_', num2str(label), '_depth_adj.png');

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
            send(pub_global_adjDepth, msg_out);

            %% binary mask
            % get global publisher
            pub_global_predictions = getGlobalPubBoudingBoxes;
        
            %disp(num2str(struct_bb_pred(label).x1));
            num_pred = size(struct_bb_pred(label).x1,1);
            
            for window = 1:num_pred
                % initialize
                img_bin = single(zeros(size(img)));
                
                x1 = struct_bb_pred(label).x1(window);
                x2 = struct_bb_pred(label).x2(window);
                y1 = struct_bb_pred(label).y1(window);
                y3 = struct_bb_pred(label).y3(window);
                
                % set mask on where the image was predicted
                img_bin(y1:y3, x1:x2) = 1;

                % write binary prediction image to data
                writeImage(msg_out, img_bin);
    
                % publish message
                send(pub_global_predictions, msg_out);
    
            end
        
        end % else

    end % if nzz

end


function sensor_msgsInfoCallback(~, msg_in)
    if getDebug
        disp("RECEIVED depth_registered/camera_info message, Doing Callback !");
    end
    
    % get global publisher
    pub_global_adjDepth_info = getGlobalPubInfo;

    % define message type
    msg_out = rosmessage('sensor_msgs/CameraInfo');
    
    % copy hole message
    msg_out = msg_in;
    
    % write message to new topic (with same name as adjusted depth)
    % this is needed for the pointcloud visualization in Rviz
    
    % publish message
    send(pub_global_adjDepth_info, msg_out);
    
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


function setGlobalPubInfo(pub)
    global xx
    xx = pub;
end

function r = getGlobalPubInfo
    global xx
    r = xx;
end


function setGlobalPubBoudingBoxes(pub)
    global y
    y = pub;
end

function r = getGlobalPubBoudingBoxes
    global y
    r = y;
end

%% global timevector of correspondant times

function setGlobalTimeCorresp
    global a
    % ONLY change this 1 line here !!! -----------------------------------
    time_struct = load('/home/andreas/Documents/DepthAdaptation/data/msg_time_vec_depth_corr.mat');  %%%%%%%%%%%
    % ONLY change this 1 line here !!! -----------------------------------
    a = time_struct.msg_time_vec_depth_corr;  %%%%%%%%%%%%%%%%%%
end

function time_vec = getGlobalTimeCorresp
    global a
    time_vec = a;
end


%% global vector with bouding box coordinates

function setGlobalBoundingBoxes
    global b
    % ONLY change this 1 line here !!! -----------------------------------
    bb_struct = load('/home/andreas/Documents/MATLAB/struct_bb_pred.mat');
    % ONLY change this 1 line here !!! -----------------------------------
    b = bb_struct.struct_bb_pred;
end

function bb = getGlobalBoundingBoxes
    global b
    bb = b;
end


%% global variable debug

function setDebug(bool)
    global d
    d = bool;
end

function r = getDebug
    global d
    r = d;
end