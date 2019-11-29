function [xy_bb_left, xy_bb_right] = getLeftRightLinesInBBwGravity(lines, x1, x2, y1, y3, depth_img, label, debug)
% select best line for left edge and right edge from all lines detected
% input:    lines: from hough transform
%           x1, x2, y1, y3: bounding box coordinates
%           debug: boolean, if true prints detected lines in bounding box
%
% output:   xy_bb_left: coordinates of start and end point of best left line
%           xy_bb_right: coordinates of start and end point of best right line
%           !!! coordinates start in bounding box top left corner with (0,0), 
%            not anymore in hole image !!!
% old function name was calculate_leftRight_lines

% initialization
max_len_left = 0;
max_len_right = 0;
line_of_interest = nan(length(lines),1);
xy_bb_left = 0;
xy_bb_right = 0;

% define borders for points of left and right lines to lie in
x_bb_2_3_left = floor(2*(x2-x1)/3); % pixel value at 2/3 of width
x_bb_1_3_right = floor((x2-x1)/3);  % pixel value at 1/3 of width

% load pose
load('data/poses_lookedUp.mat'); % loaded as poses
% convert translation from [m] to [mm]
poses(1:3,4,label) = poses(1:3,4,label)*1000;
T_wc = poses(:,:,label);

for k = 1:length(lines)

    % initialization
    points_z = zeros(2,1);
    kernel = 5;
    
    while any(points_z == 0)
        points_x = [lines(k).point1(1); lines(k).point2(1)];
        points_x = points_x + x1;
        points_y = [lines(k).point1(2); lines(k).point2(2)];
        points_y = points_y + y1;
        points_z = [kernelMedian(kernel, points_x(1), points_y(1), depth_img);
                    kernelMedian(kernel, points_x(2), points_y(2), depth_img)];

        % increase kernel size
        if any(points_z == 0)
            kernel = kernel + 1;
            disp(strcat('Increased kernel: ', num2str(kernel)));
        end
    end
    %%
%     figure(1)
%     visualize_depth_png(depth_img)  % millimeters
%     hold on
%     plot([lines(k).point1(1)+x1;lines(k).point2(1)+x1], [lines(k).point1(2)+y1;lines(k).point2(2)+y1],'LineWidth',2);
   

    %%
    P_line = [points_x, points_y, points_z];
    [X_c_line, Y_c_line, Z_c_line] = pixelNormalView2camCoordinate(P_line);
    P_c_line = [X_c_line'; Y_c_line'; Z_c_line'];

    P =   T_wc * [P_c_line; ones(1,size(P_c_line,2))];
    P_w_line = P(1:end-1,:);

    % vectors
    V_line = P_w_line(:,1)-P_w_line(:,2);
    V_z = [0;0;1];
    angle2zAxis = rad2deg(acos( (V_line'*V_z) / (norm(V_line))));
    lines(k).angle2z = angle2zAxis;
    
    % vertical line
    if lines(k).angle2z < 2
        
        %line_left if start and end point is in left 2/3 of bounding box
        if lines(k).point1(1) < x_bb_2_3_left && lines(k).point2(1) < x_bb_2_3_left
            len = norm(lines(k).point1 - lines(k).point2);
            if (len > 1.1*max_len_left) && (len > round(1/3*(y3-y1)))
                max_len_left = len;
                % endpoint of longest top line
                xy_bb_left = [lines(k).point1; lines(k).point2];
            end
            if debug == true
                figure(11)
                hold on
                plot([lines(k).point1(1);lines(k).point2(1)], [lines(k).point1(2);lines(k).point2(2)],'LineWidth',2);
            end
        %line_right if start and end point is in right 2/3 of bounding box
        elseif lines(k).point1(1) > x_bb_1_3_right && lines(k).point2(1) > x_bb_1_3_right
            len = norm(lines(k).point1 - lines(k).point2);
            if (len > 1.1*max_len_right) && (len > round(1/3*(y3-y1)))
                max_len_right = len;
                % endpoint of longest bot line
                xy_bb_right = [lines(k).point1; lines(k).point2];
            end
            if debug == true
                figure(11);
                hold on
                plot([lines(k).point1(1);lines(k).point2(1)], [lines(k).point1(2);lines(k).point2(2)],'LineWidth',2);
                pause(0.2)
            end
        end
        
    end
end

end

