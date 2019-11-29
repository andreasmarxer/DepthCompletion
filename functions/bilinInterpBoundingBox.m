function f = bilinInterpBoundingBox(x, y, p1, p2, p3, p4)


%% basic case where we have a depth at each of the 4 corner points

% x coordinates
x1 = p1(1); x2 = p2(1); x3 = p3(1); x4 = p4(1);
% y coordinates
y1 = p1(2); y2 = p2(2); y3 = p3(2); y4 = p4(2);
% depth values
f1 = p1(3); %kernelMedian(kernel, x1, y1, depth_img);  %takes median of 3x3 arround pixel neglecting zeros 
f2 = p2(3); %kernelMedian(kernel, x2, y2, depth_img);  %takes median of 3x3 arround pixel neglecting zeros 
f3 = p3(3); %kernelMedian(kernel, x3, y3, depth_img);  %takes median of 3x3 arround pixel neglecting zeros 
f4 = p4(3); %kernelMedian(kernel, x4, y4, depth_img);  %takes median of 3x3 arround pixel neglecting zeros 


%% else bilinear interpolation

prefactor = 1/((x2-x1)*(y4-y1));

X = [x2-x, x-x1];
Y = [y4-y; y-y1];
F = [f1, f4; f2, f3];

f = prefactor * X * F * Y;

% convert back to integer with scale mm in depth_img;
f = uint16(f);

end