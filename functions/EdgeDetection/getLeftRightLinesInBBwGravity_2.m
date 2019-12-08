function [xy_bb_left, xy_bb_right] = getLeftRightLinesInBBwGravity_2(lines, x1, x2, y1, y3, depth_img, debug)
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
xy_bb_left = nan(2,2);
xy_bb_right = nan(2,2);

% define borders for points of left and right lines to lie in
x_bb_2_3_left = floor(2*(x2-x1)/3); % pixel value at 2/3 of width
x_bb_1_3_right = floor((x2-x1)/3);  % pixel value at 1/3 of width


for k = 1:length(lines)

    % vertical line
    if lines(k).angle2gravity < 5
        
        %line_left if start and end point is in left 2/3 of bounding box
        if lines(k).point1(1) < x_bb_2_3_left && lines(k).point2(1) < x_bb_2_3_left
            len = norm(lines(k).point1 - lines(k).point2);
            if (len > 1.1*max_len_left) && (len > round(1/3*(y3-y1)))
                max_len_left = len;
                % endpoint of longest top line
                xy_bb_left = [lines(k).point1; lines(k).point2];
                % Debugging
                if debug == true
                    figure_id = 10;
                    figure_title = 'all longest vertical lines in plane';
                    plot_lines(figure_id, figure_title, x1, x2, y1, y3, lines(k), debug, depth_img)
                    pause(0.2);
                end
            end
        %line_right if start and end point is in right 2/3 of bounding box
        elseif lines(k).point1(1) > x_bb_1_3_right && lines(k).point2(1) > x_bb_1_3_right
            len = norm(lines(k).point1 - lines(k).point2);
            if (len > 1.1*max_len_right) && (len > round(1/3*(y3-y1)))
                max_len_right = len;
                % endpoint of longest bot line
                xy_bb_right = [lines(k).point1; lines(k).point2];
                % Debugging
                if debug == true
                    figure_id = 10;
                    figure_title = 'all longest vertical lines in plane';
                    plot_lines(figure_id, figure_title, x1, x2, y1, y3, lines(k), debug, depth_img)
                    pause(0.2);
                end
            end
        end 
        
    end
    
end

end

