
function [xy_bb_left, xy_bb_right] = getLeftRightLinesInBB(lines, x1, x2, y1, y3, debug)
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

for k = 1:length(lines)

    % only lines that have an theta angle < 25° = "vertical lines"
    line_of_interest(k) = 0 < abs(lines(k).theta) && abs(lines(k).theta) < 25;
    if line_of_interest(k) == true

        %line_left
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
        %line_right
        elseif lines(k).point1(1) > x_bb_1_3_right && lines(k).point2(1) > x_bb_1_3_right
            len = norm(lines(k).point1 - lines(k).point2);
            if (len > 1.1*max_len_right) && (len > round(1/3*(y3-y1)))
                max_len_right = len;
                % endpoint of longest bot line
                xy_bb_right = [lines(k).point1; lines(k).point2];
            end
            if debug == true
                figure(11)
                hold on
                plot([lines(k).point1(1);lines(k).point2(1)], [lines(k).point1(2);lines(k).point2(2)],'LineWidth',2);
            end
        end
    end

end

end

