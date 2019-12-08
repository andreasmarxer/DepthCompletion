function med = kernelMedian(k, x, y, depth_img)

if mod(k,2)==0
    error('ERROR: Kernel must be odd size for this application!');
end

height = size(depth_img,1);
width = size(depth_img,2);

% calculate dimension from center kernel pixel to end of kernel
k_half = (k-1)/2;

%% shift x,y away if too near at boarder (kind of padding)
% x coordinate is too near at border -> shift at req. distance
if x<=k_half 
    x = k_half+1;
elseif x>width-k_half
    x = width-k_half;
end
% y coordinate is too near at border -> shift at req. distance
if y<=k_half
    y = k_half+1;
elseif y>height-k_half
    y = height-k_half;
end

%% take kernel at position from depth image
img_kernel = depth_img(y-k_half:y+k_half, x-k_half:x+k_half);

% get the locations of nonzero 
nonzero_idx = img_kernel>0;
img_kernel_non_zero = img_kernel(nonzero_idx);

% median of the non zero entries
med = median(img_kernel_non_zero);

end