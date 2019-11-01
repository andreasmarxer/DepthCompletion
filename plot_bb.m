function [x1, y1, x2, y2, x3, y3, x4, y4 ] = plot_bb(left_x_vec, top_y_vec, width_vec, height_vec, img)

    for window = 1:size(left_x_vec)
        [x1, y1, x2, y2, x3, y3, x4, y4 ] = convertToBBCoords(left_x_vec(window), top_y_vec(window), width_vec(window), height_vec(window), img);

        hold on
        plot(x1, y1, 'rx')
        hold on
        plot(x2, y2, 'gx')
        hold on
        plot(x3, y3, 'cx')
        hold on
        plot(x4, y4, 'wx')
        hold off
    end
    
end

