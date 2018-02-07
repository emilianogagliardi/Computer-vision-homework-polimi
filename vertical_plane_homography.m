function Hrec = vertical_plane_homography(K, R, t)
    % given intrinsic and extrinsic parameters of a Pinhole model find the
    % mapping from point on the image to point in the real world belonging
    % to the vertical plane XZ in the world reference frame represented by
    % the extrinsic parameters
    P = K*[R, t];
    H = [P(:, 1), P(:, 3), P(:, 4)];
    Hrec = inv(H);
end