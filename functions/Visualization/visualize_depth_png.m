function visualize_depth_png(img_png)
    img_scaled = 10*img_png;    %scale that uint16 range is better used
    imshow(img_scaled, 'colormap', jet);
    originalSize = get(gca, 'Position');
    c = colorbar;
    c.Label.String = 'Depth in [10⁻4m]';
    set(gca, 'Position', originalSize); % set current axis to size before colormap
end
