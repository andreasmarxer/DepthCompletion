function plot_lines(figure_id, figure_title, x1, x2, y1, y3, lines, debug, img)

if debug == true
    
    BW= edge(img(y1:y3, x1:x2), 'Canny');
    B = false(size(img,1), size(img,2));
    B(y1:y3,x1:x2) = BW;
    % plotting

    figure(figure_id)
    imshow(B)
    title(figure_title)
    pause(0.5)
    % show all lines
    for k = 1:length(lines)
        hold on;
        plot([lines(k).point1(1)+x1;lines(k).point2(1)+x1], [lines(k).point1(2)+y1;lines(k).point2(2)+y1],'LineWidth',2);
        
    end
   
end

end