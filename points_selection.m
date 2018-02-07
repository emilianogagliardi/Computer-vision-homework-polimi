function selected_pts = points_selection(im, corners, im_title)
    fig = figure();
    imshow(im);
    hold on   
    plot(corners);
    title(im_title);
    [x, y] = getpts();
    close(fig);
    pts = corners.Location;
    selected_pts = [];
    for ii = 1 : length(x)
        min_d = inf;
        nearest = pts(1, :);
        for jj = 1 : size(pts, 1)
            d = norm([x(ii), y(ii)] - pts(jj, :), 2);
            if d < min_d
                min_d = d;
                nearest = pts(jj, :);
            end
        end
        selected_pts = [selected_pts; [nearest, 1]];
    end
end

