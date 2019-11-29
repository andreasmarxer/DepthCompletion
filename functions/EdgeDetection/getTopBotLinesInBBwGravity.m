function [xy_bb_top, xy_bb_bot] = getTopBotLinesInBB(lines, y1, y3, x1, x2, depth_img, label, debug)
% select best line for top edge and bottom edge from all lines detected
% input:    lines: from hough transform
%           x1, x2, y1, y3: bounding box coordinates
%           debug: boolean, if true prints detected lines in bounding box
%
% output:   xy_bb_top: coordinates of start and end point of best top line
%           xy_bb_bot: coordinates of start and end point of best left line
%           !!! coordinates start in bounding box top left corner with (0,0), 
%            not anymore in hole image !!!
% old function name was calculate_topBot_lines

% initialization
max_len_top = 0;
max_len_bot = 0;
xy_bb_top = 0;
xy_bb_bot = 0;

y_bb_1_3_top = floor((y3-y1)/3); % no + y1 due to the fact that this is in BB coords
y_bb_2_3_bot = floor(2*(y3-y1)/3);

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
    
    % horizontal line
    if lines(k).angle2z < 92 && lines(k).angle2z > 88
        %line_top if start and end point is in top 1/3 of bounding box
        if lines(k).point1(2) < y_bb_1_3_top && lines(k).point2(2) < y_bb_1_3_top
            len = norm(lines(k).point1 - lines(k).point2);
            if (len > 1.1*max_len_top)
                max_len_top = len;
                % endpoint of longest top line
                xy_bb_top = [lines(k).point1; lines(k).point2];
            end
            % Debugging
            if debug == true
                figure(12)
                hold on
                plot([lines(k).point1(1);lines(k).point2(1)], [lines(k).point1(2);lines(k).point2(2)],'LineWidth',2);
            end
        %line_bottom if start and end point is in bottom 1/3 of bounding box
        elseif lines(k).point1(2) > y_bb_2_3_bot && lines(k).point2(2) > y_bb_2_3_bot
            len = norm(lines(k).point1 - lines(k).point2);
            if (len > 1.1*max_len_bot)
                max_len_bot = len;
                % endpoint of longest bot line
                xy_bb_bot = [lines(k).point1; lines(k).point2];
            end
            % Debuging
            if debug == true
                figure(12);
                hold on
                plot([lines(k).point1(1);lines(k).point2(1)], [lines(k).point1(2);lines(k).point2(2)],'LineWidth',2);
                pause(0.2)
            end
        end
  
    end
    


end

end
