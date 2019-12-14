%% params
label = 2;
rotation_back = true;
print = false;
%%

path = '/home/andreas/Documents/DepthAdaptation/';
filename = strcat(path, 'asl_window_depth_',  num2str(label), '_adj.png');

%%

img_png = imread(filename); % PNG image: uint16, [mm]

img_png = single(img_png); % backtransform to single

zero_idx = img_png < 0.0001;
img_png(zero_idx) = NaN; % backtransform zeros to NaN

img_png = img_png/1000; % backtransform to [m]

if rotation_back == true
    img_png = imrotate(img_png, 180);
end
                
msg_test = rosmessage('sensor_msgs/Image');
msg_test.Encoding = '32FC1';
msg_test.Step = 2560;

writeImage(msg_test, img_png);

disp('-Written');
msg_test
data_write = msg_test.Data;
