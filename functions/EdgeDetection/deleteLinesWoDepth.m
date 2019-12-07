function lines = deleteLinesWoDepth(lines, debug)

% initialization
depth_not_avail = zeros(length(lines),1);

for k = 1:length(lines)
   
    depth_not_avail(k) = isnan(lines(k).angle2gravity);
    
end
% invert logicals
depth_avail = ~depth_not_avail;
% delete the lines without depth measurement
lines = lines(depth_avail);

end