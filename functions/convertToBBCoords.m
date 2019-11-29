function [x1, y1, x2, y2, x3, y3, x4, y4 ] = convertToBBCoords(left_x, top_y, width, height, img)
% convert YOLOv3 format to corner point in [px] of bounding box
% input:    one bounding box in YOLOv3 prediction format
%           left_x, top_y: coordinates of top left point in [px]
%           width, height: width and height of bb in [px]
% output:   coordnates of corners from bounding box in pixel
%           convention: 1=top left, 2=top right, 3=bottom right, 4=bottom left !!!

% top left
x1 = left_x;
y1 = top_y;
[x1,y1] = imgBorderCheck(x1,y1, img);

% top right
x2 = left_x + width;
y2 = top_y;
[x2,y2] = imgBorderCheck(x2,y2, img);

% bottom right
x3 = x2;
y3 = top_y + height;
[x3,y3] = imgBorderCheck(x3,y3, img);

% bottom left
x4 = x1;
y4 = y3;
[x4,y4] = imgBorderCheck(x4,y4, img);



%% Check if corners are at unplottable border point
    function [x,y] = imgBorderCheck(x,y, img)

    img_height = size(img,1);
    img_width = size(img,2);

        if x<1
            x=1; % can plot from 1 on
        elseif x>img_width
            x=img_width;
        end

        if y<1
            y=1; % can plot from 1 on
        elseif y>img_height
            y=img_height;
        end

    end

end