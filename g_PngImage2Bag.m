%% params
frame = 2;
rotation_back = true;
print = false;
save_img = false;
%%

path = '/home/andreas/Documents/ASL_window_dataset/depth_images_adj/';
filename = strcat(path, 'asl_window_', num2str(label), '_depth_adj.png');

%%
img = imread(filename); % PNG image: uint16, [mm]
img = img/1000; % backtransform to [m]
img(img == 0)=NaN; % backtransform zeros to NaN

if rotation_back == true
    img = imrotate(img, 180);
end

img = single(img); % backtransform to single

% NEED TO GENERATE MESSAGE BEFORE !!!!!!!!!!!!!!!!!!!!!!!
writeImage(msg, img);
