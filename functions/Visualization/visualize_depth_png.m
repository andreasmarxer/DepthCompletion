function visualize_depth_png(img_png)
% plot the png depth image with factor 10 to get a good color visualization
% input:    img_png: array of depth image in [mm]
%
% output:   none, plots figure

    img_scaled = 10*img_png;    %scale that uint16 range is better used
    imshow(img_scaled, 'colormap', jet);
    originalSize = get(gca, 'Position');
    c = colorbar;
    c.Label.String = 'Depth in [10⁻4m]';
    set(gca, 'Position', originalSize); % set current axis to size before colormap
    
end
