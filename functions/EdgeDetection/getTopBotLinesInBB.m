function [xy_bb_top, xy_bb_bot] = getTopBotLinesInBB(lines, y1, y3, x1, x2, debug)
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
line_of_interest = nan(length(lines),1);
xy_bb_top = 0;
xy_bb_bot = 0;


y_bb_1_3_top = floor((y3-y1)/3); % no + y1 due to the fact that this is in BB coords
y_bb_2_3_bot = floor(2*(y3-y1)/3);

for k = 1:length(lines)

    % only lines that are 45 or horizontal, not interessted in vertical
    line_of_interest(k) = 45 < abs(lines(k).theta) && abs(lines(k).theta) < 90;
    if line_of_interest(k) == true

        %line_top
        if lines(k).point1(2) < y_bb_1_3_top && lines(k).point2(2) < y_bb_1_3_top
            len = norm(lines(k).point1 - lines(k).point2);
            if (len > 1.1*max_len_top)
                max_len_top = len;
                % endpoint of longest top line
                xy_bb_top = [lines(k).point1; lines(k).point2];
            end
            % Debugging
            if debug == true
                figure(10)
                hold on
                plot([lines(k).point1(1);lines(k).point2(1)], [lines(k).point1(2);lines(k).point2(2)],'LineWidth',2);
            end
        %line_bottom
        elseif lines(k).point1(2) > y_bb_2_3_bot && lines(k).point2(2) > y_bb_2_3_bot
            len = norm(lines(k).point1 - lines(k).point2);
            if (len > 1.1*max_len_bot)
                max_len_bot = len;
                % endpoint of longest bot line
                xy_bb_bot = [lines(k).point1; lines(k).point2];
            end
            % Debuging
            if debug == true
                figure(10)
                hold on
                plot([lines(k).point1(1);lines(k).point2(1)], [lines(k).point1(2);lines(k).point2(2)],'LineWidth',2);
            end
        end
    end

end

end
