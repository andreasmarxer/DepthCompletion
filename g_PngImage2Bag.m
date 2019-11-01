%% params
label = 2;
rotation_back = true;
print = false;
save_img = false;
%%

path = '/home/andreas/Documents/ASL_window_dataset/depth_images_adj/';
filename = strcat(path, 'asl_window_', num2str(label), '_depth_adj.png');

%%

img = imread(filename); % PNG image: uint16, [mm]

img = single(img); % backtransform to single

zero_idx = img < 0.0001;
img(zero_idx) = NaN; % backtransform zeros to NaN

img = img/1000; % backtransform to [m]

if rotation_back == true
    img = imrotate(img, 180);
end
        
        


% NEED TO GENERATE MESSAGE BEFORE !!!!!!!!!!!!!!!!!!!!!!!
writeImage(msg, img);
