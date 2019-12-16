function [x1, y1, x2, y2, x3, y3, x4, y4 ] = plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, img)
% plots predicted bounding box corner points
% input:    bounding boxes in YOLOv3 prediction format
%           vector length is number of predictions in image
%           left_x_vec, top_y_vec: coordinates of top left point in [px]
%           width_vec, height_vec: width and height of bb in [px]
%
% output:   none, plots bounding box corners in figure

    for window = 1:size(left_x_vec)
        % convert YOLOv3 format to corner point
        [x1, y1, x2, y2, x3, y3, x4, y4 ] = convertToBBCoords(left_x_vec(window), top_y_vec(window), width_vec(window), height_vec(window), img);

        hold on
        plot(x1, y1, 'rx', 'LineWidth', 2)
        hold on
        plot(x2, y2, 'gx', 'LineWidth', 2)
        hold on
        plot(x3, y3, 'cx', 'LineWidth', 2)
        hold on
        plot(x4, y4, 'wx', 'LineWidth', 2)
        hold off
    end
    
end

