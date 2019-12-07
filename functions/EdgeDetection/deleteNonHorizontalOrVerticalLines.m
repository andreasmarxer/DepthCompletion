function lines = deleteNonHorizontalOrVerticalLines(lines, debug)    

% initialization
angle_satisfied = logical(zeros(1, length(lines)));

for k = 1:length(lines)
    
    angle = lines(k).angle2gravity;
    angle_satisfied(k) = angle < 5 || (angle > 85 && angle < 95);   
    
end

% delete the lines without a horizontal or vertical angle

lines = lines(angle_satisfied);
      
end