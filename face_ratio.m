function ratio = face_ratio(Hrec, face_edges)
    % this function uses the lines of the image and the reconstruction
    % matrix to compute the ratio between the sides of a face given its
    % edges
    
    % lines of the face
    h_lines_im = [  face_edges.top; ...
                    face_edges.right; ...
                    face_edges.bottom; ...
                    face_edges.left];

    % compute the ratio between short and long side ratio := short / long
    % this can be done by finding the vertices in the reconstructed image
    % apply the reconstruction to the lines
    h_lines_rec = h_lines_im / Hrec; 
    % intersect the lines to obtain the points
    up_right_rec = cross(h_lines_rec(1, :), h_lines_rec(2, :));
    up_left_rec = cross(h_lines_rec(4, :), h_lines_rec(1, :));
    bottom_left_rec = cross(h_lines_rec(3, :), h_lines_rec(4, :));
    % compute the leght in the similarity to obtain the ratio
    up_right_rec = up_right_rec / up_right_rec(3);
    up_left_rec = up_left_rec / up_left_rec(3);
    bottom_left_rec = bottom_left_rec / bottom_left_rec(3);
    long = norm(up_left_rec(1:2) - bottom_left_rec(1:2), 2);
    short = norm(up_left_rec(1:2) - up_right_rec(1:2), 2);
    ratio = short / long;
end