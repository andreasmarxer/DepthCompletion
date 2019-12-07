function [lines, gravity_cam] = convert2cameraAlignedGravity(lines, x1, y1, depth_img, label, dataset, debug)

% load pose
pose_filepath = strcat('data/poses_lookedUp_', dataset ,'_optimized.mat');
load(pose_filepath); % loaded as poses
% convert translation from [m] to [mm]
poses(1:3,4,label) = poses(1:3,4,label)*1000;
T_wc = poses(:,:,label);
R_wc = T_wc(1:3,1:3);

gravity_world = [0; 0; 1];
gravity_cam = R_wc' * gravity_world;

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
    % write to lines struct
    lines(k).point1_px = [points_x(1), points_y(1), points_z(1)];
    lines(k).point2_px = [points_x(2), points_y(2), points_z(2)];

    if any(points_z == 0)
        lines(k).angle2gravity = nan; % set to value that we don't take into account
        continue; % with next for loop
    end
    
    P_line = [points_x, points_y, points_z];
    [X_c_line, Y_c_line, Z_c_line] = pixelNormalView2camCoordinate(P_line);
    P_c_line = [X_c_line'; Y_c_line'; Z_c_line'];
    % write to lines struct
    lines(k).point1_cam = [X_c_line(1), Y_c_line(1), Z_c_line(1)];
    lines(k).point2_cam = [X_c_line(2), Y_c_line(2), Z_c_line(2)];
    
    
    % vectors camera frame
    V_line = P_c_line(:,1)-P_c_line(:,2);
    V_z = gravity_cam;
    angle2zAxis = rad2deg(acos( (V_line'*V_z) / (norm(V_line))));
    % write to lines struct
    lines(k).angle2gravity = angle2zAxis;
    
    lines(k).vertical = false;
    lines(k).horizontal = false;
    
    if angle2zAxis < 5
        lines(k).vertical = true;
    elseif (angle2zAxis > 85 && angle2zAxis < 95)
        lines(k).horizontal = true;
    end
    
end

end