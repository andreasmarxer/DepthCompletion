function adjDepth2PngImage(depth_img_adj, label, rotate_back)
% save adjusted depth image as PNG image in path specified under filepath
%
% input:    depth_img_adj:  adjusted depth image in uint16 format
%           label:          number for labelling png image
%           rotate_back:    boolean, set true for rotate back depth images
%                           to overhead view -> needed for Open3D
%
% output:   none, png image is saved under filepath

% make changes here -------------------------------------------------------
type = 'depth_adj';
filepath =  '/home/andreas/Documents/ASL_window_dataset/depth_images_adj/temp/';
% -------------------------------------------------------------------------

if rotate_back == true
    img = imrotate(depth_img_adj, 180);
else
    img = depth_img_adj;
end

filename = strcat('asl_window_', num2str(label), '_', num2str(type), '.png');
imwrite(img, strcat(filepath, filename), 'fmt', 'png');

end