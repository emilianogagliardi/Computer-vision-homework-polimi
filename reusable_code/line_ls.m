function line = line_ls(h_pt)
    % takes in input a set of points in homogeneous coordinates and find
    % the total least square solution
    % input lines must be row vectors
    
    % if only two points directly cross them, otherwise will lead to ill
    % conditioned matrices
    if size(h_pt, 1) == 2
        line = cross(h_pt(1, :), h_pt(2, :));
    else
        x = h_pt(:, 1) ./ h_pt(:, 3);
        y = h_pt(:, 2) ./ h_pt(:, 3);
        % compute the elements of the matrix A
        mean_of_squares_x = mean(x.^2);
        square_of_mean_x = mean(x)^2;
        mean_of_squares_y = mean(y.^2);
        square_of_mean_y = mean(y)^2;
        mean_of_products = mean(x.*y);
        product_of_means = mean(x)*mean(y);
        A = [mean_of_squares_x  -   square_of_mean_x, ...
            mean_of_products    -   product_of_means;
            mean_of_products    -   product_of_means, ...
            mean_of_squares_y   -   square_of_mean_y];
        % find the eigenvector associated to the smaller eigenvalue of the
        % matrix A, its component will be sin theta, cos theta, where theta is
        % the angle the perpendicular of the line we are fitting passing
        % through the origin forms with the horizontal axis
        [V,~] = eigs(A,1,'SM');
        a = V(1);
        b = V(2);
        % c is the rho value, thus the perpendicular distance between the
        % origin and the line we are fitting
        c = - a * mean(x) - b * mean(y);
        line = [a/b, 1, c/b];
    end
end
        