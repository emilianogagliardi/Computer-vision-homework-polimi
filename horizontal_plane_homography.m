function H = horizontal_plane_homography(left_face_edges, Hrec)
    % Compute the transformation that brings the horizontal plane from the 
    % image to the real world, where the reference system is placed on
    % the left face
    
    % intersect the lines to compute the vertices of the left face in the
    % image
    up_right_im = cross(left_face_edges.right, left_face_edges.top);
    bottom_right_im = cross(left_face_edges.bottom, left_face_edges.right);
    bottom_left_im = cross(left_face_edges.bottom, left_face_edges.left);
    up_left_im = cross(left_face_edges.top, left_face_edges.left);
    
    ratio = face_ratio(Hrec, left_face_edges);

    % Fit the homography that brings from the image to the world. The
    % real shape (ratio) is known from the reconstruction and the lenght 
    % of the long side is given
    LONG_REAL = 243;
    SHORT_REAL = LONG_REAL * ratio;
    % by setting the points in the following way we are fixing the
    % reference system of the object on the left face. The positive 
    % directions can be inferred by the sign of the values
    real_points = [0 0; 0 LONG_REAL; SHORT_REAL 0; SHORT_REAL LONG_REAL];
    % points obtained in the image
    image_points= [bottom_left_im(1:2) / bottom_left_im(3);
                   up_left_im(1:2)/ up_left_im(3);
                   bottom_right_im(1:2) / bottom_right_im(3);
                   up_right_im(1:2) / up_right_im(3)];
    % homography
    tform = fitgeotrans(image_points, real_points, 'projective');
    H = tform.T';
end