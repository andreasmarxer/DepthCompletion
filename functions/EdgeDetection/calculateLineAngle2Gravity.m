function lines = calculateLineAngle2Gravity(lines, x1, y1, depth_img, label, dataset, debug)

% load pose
pose_filepath = strcat('data/poses_lookedUp_', dataset ,'.mat');
load(pose_filepath); % loaded as poses
% convert translation from [m] to [mm]
poses(1:3,4,label) = poses(1:3,4,label)*1000;
T_wc = poses(:,:,label);
    
for k = 1:length(lines)
    % initialization
    points_z = zeros(2,1);
    kernel = 5;
    
    points_x = [lines(k).point1(1); lines(k).point2(1)];
    points_x = points_x + x1;
    points_y = [lines(k).point1(2); lines(k).point2(2)];
    points_y = points_y + y1;
    points_z = [kernelMedian(kernel, points_x(1), points_y(1), depth_img);
                kernelMedian(kernel, points_x(2), points_y(2), depth_img)];

    
    if any(points_z == 0)
        lines(k).angle2z = 999; % set to value that we don't take into account
        continue; % with next for loop
    end
    
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
    
end

end