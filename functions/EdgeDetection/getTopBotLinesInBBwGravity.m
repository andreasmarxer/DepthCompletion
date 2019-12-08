function [xy_bb_top, xy_bb_bot] = getTopBotLinesInBB(lines, x1, x2, y1, y3, debug)
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
    
for k = 1:length(lines)
    
    % horizontal line
    if lines(k).angle2z < 94 && lines(k).angle2z > 86
        
        %line_top if start and end point is in top 1/3 of bounding box
        if lines(k).point1(2) < y_bb_1_3_top && lines(k).point2(2) < y_bb_1_3_top
            len = norm(lines(k).point1 - lines(k).point2);
            if (len > 1.1*max_len_top) %&& (len > round(1/5*(x2-x1)))
                max_len_top = len;
                % endpoint of longest top line
                xy_bb_top = [lines(k).point1; lines(k).point2];
                % Debugging
                if debug == true
                    figure(12)
                    hold on
                    plot([lines(k).point1(1);lines(k).point2(1)], [lines(k).point1(2);lines(k).point2(2)],'LineWidth',2);
                end
            end
        %line_bottom if start and end point is in bottom 1/3 of bounding box
        elseif lines(k).point1(2) > y_bb_2_3_bot && lines(k).point2(2) > y_bb_2_3_bot
            len = norm(lines(k).point1 - lines(k).point2);
            if (len > 1.1*max_len_bot) %&& (len > round(1/5*(x2-x1)))
                max_len_bot = len;
                % endpoint of longest bot line
                xy_bb_bot = [lines(k).point1; lines(k).point2];
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

end
