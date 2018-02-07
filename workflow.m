clear
clc
close all

% set interactive to true if you want to manually select the lines, 
% if interactive is set to false the algorithms will use hardcoded lines
interactive = false;

im = imread('Input image.jpeg');

% extract the lines and show the result
l_faces = extract_lines_faces(im);
l_floor = extract_lines_floor(im);

selected_lines = lines_selection(interactive, l_faces, l_floor, im);

norm_factor = max((size(im)));
norm_matrix = diag([1/norm_factor, 1/norm_factor, 1]);

% normalized lines
lhl_n = selected_lines.lhl / norm_matrix;
lhs_n = selected_lines.lhs / norm_matrix;
rhl_n = selected_lines.rhl / norm_matrix;
rhs_n = selected_lines.rhs / norm_matrix;
fl1_n = selected_lines.fl1   / norm_matrix;
fl2_n = selected_lines.fl2   / norm_matrix;
vl_n = selected_lines.vl   / norm_matrix;

% apply the affine reconstruction and show the image
Haffine_n = affine_rectification(lhl_n, lhs_n, rhl_n, rhs_n, fl1_n, fl2_n);
linf_n = Haffine_n(3, :);
% undo the normalization to show the image affinely reconstructed
linf = linf_n * norm_matrix;
Haffine = [1 0 0; 0 1 0; linf];
T_aff = projective2d(Haffine');
affine_im = imwarp(im, T_aff);
figure, imshow(affine_im);
title ('horizontal plane affine reconstruction');

% generate a list of perpendicular pairs as input for the shape 
% reconstruction algoritm.
% the reconstruction is applyed on the already rectified lines
perp_pairs_rec = [];
% left face
for ii = 1:size(lhl_n, 1)
    for jj = 1:size(lhs_n, 1)
        perp_pairs_rec = [perp_pairs_rec struct('l', lhl_n(ii, :) / ...
            Haffine_n, 'm', lhs_n(jj, :)/Haffine_n)];
    end
end
% right face
for ii = 1:size(rhl_n, 1)
    for jj = 1:size(rhs_n, 1)
        perp_pairs_rec = [perp_pairs_rec struct('l', rhl_n(ii, :) / ...
            Haffine_n, 'm', rhs_n(jj, :) / Haffine_n)];
    end
end
% floor
for ii = 1:size(fl1_n, 1)
    for jj = 1:size(fl2_n, 1)
        perp_pairs_rec = [perp_pairs_rec struct('l', fl1_n(ii, :) / ...
            Haffine_n, 'm', fl2_n(jj, :) / Haffine_n)];
    end
end
H_aff_to_shape = reconstruct2D(perp_pairs_rec);
% the reconstruction matrix is the composition of the two 
Hrec = H_aff_to_shape * Haffine;

% apply the euclidean reconstruction and show the image
T = projective2d(Hrec');
reconstructed_im = imwarp(im, T);
figure, imshow(reconstructed_im);
title('horizontal plane shape reconstruction');

% localization of the right face with respect to the left face
% The rotation matrix and the vector are expressed in a
% refererence frame placed on the bottom left points of the left face, with
% y axis on the long edge and x axis on the short edge
% The rotation matrix represent the rotation of one long edge of the right
% face with respect to one long edge of the left face
[R_r_wrt_l, t_r_wrt_l] = loc_right_face_wrt_left(Hrec, ...
    selected_lines.left_edges, selected_lines.right_edges, ...
    selected_lines.lhl, selected_lines.rhl);
% place the reference system on the left face
LONG_REAL = 243;
SHORT_REAL = LONG_REAL * face_ratio(Hrec, selected_lines.left_edges);
left_face = [0 0 0; 0 LONG_REAL 0; SHORT_REAL 0 0; SHORT_REAL LONG_REAL 0];
SHORT_REAL_RIGHT = LONG_REAL * face_ratio(Hrec, selected_lines.right_edges);
right_face = [0 0 0; 0 LONG_REAL 0; SHORT_REAL_RIGHT 0 0; ...
    SHORT_REAL_RIGHT LONG_REAL 0] * R_r_wrt_l' + t_r_wrt_l';
pcshowpair(pointCloud(left_face), pointCloud(right_face), 'MarkerSize', ...
    200);
title('relative position of the two faces in the real world');

% camera calibration
% generate the list of perpendicular pairs of vanishing points to pass to 
% the calibration algorithm
lhl_vp_n = point_ls(lhl_n);
lhs_vp_n = point_ls(lhs_n);
rhl_vp_n = point_ls(rhl_n);
rhs_vp_n = point_ls(rhs_n);
vl_vp_n = point_ls(vl_n);
fl1_vp_n = point_ls(fl1_n);
fl2_vp_n = point_ls(fl2_n);
perp_pairs_vp = [   struct('v1', lhl_vp_n, 'v2', lhs_vp_n), ...
                    struct('v1', rhl_vp_n, 'v2', rhs_vp_n), ...
                    struct('v1', fl1_vp_n,  'v2', vl_vp_n),  ...
                    struct('v1', fl2_vp_n,  'v2', vl_vp_n),  ...
                    struct('v1', fl1_vp_n,  'v2', fl2_vp_n),  ...
                    struct('v1', lhl_vp_n, 'v2', vl_vp_n),  ...
                    struct('v1', lhs_vp_n, 'v2', vl_vp_n),  ...
                    struct('v1', rhl_vp_n, 'v2', vl_vp_n),  ... 
                    struct('v1', rhs_vp_n, 'v2', vl_vp_n)];
K_n = calibration(perp_pairs_vp);
% undo normalization
K = norm_matrix \ K_n;

% localize the camera wrt the left face
real_to_im_horiz_plane_H = inv ( ...
    horizontal_plane_homography(selected_lines.left_edges, Hrec));
[R_c_wrt_l, t_c_wrt_l] = camera_localization_planar(K, ...
  real_to_im_horiz_plane_H);
figure();
pcshowpair(pointCloud(left_face), pointCloud(right_face), 'MarkerSize', ...
    200);
hold on
plotCamera('location', t_c_wrt_l, 'orientation', R_c_wrt_l', 'size', 20);
title('camera localized with respect to the left face');

% localize the camera with respect the right face
[R_c_wrt_r, t_c_wrt_r] = loc_cam_wrt_right_face(R_c_wrt_l, t_c_wrt_l, ...
    R_r_wrt_l, t_r_wrt_l);
% place the reference system on the right face
right_face2 = [0 0 0; 0 LONG_REAL 0; SHORT_REAL_RIGHT 0 0; ...
    SHORT_REAL_RIGHT LONG_REAL 0];
left_face2 = right_face2 * R_r_wrt_l - (R_r_wrt_l'* t_r_wrt_l)';
figure();
pcshowpair(pointCloud(left_face2), pointCloud(right_face2), ...
    'MarkerSize', 200);
hold on
plotCamera('location', t_c_wrt_r, 'orientation', R_c_wrt_r', 'size', 20);
title('camera localized with respect to the right face');

% the reconstruction of the vertical faces is done unsing the extrinsic
% parameters from the localization of the faces with respect to the camera
% and from the calibration. We have the Pinhole model where the world
% reference frame is the the left face frame and the one in which the world
% reference frame is the right face frame. So it is possible to obtain a
% mapping from the real world the the vertical plane (for each face) and
% then invert it to reconstruct the faces
H_left = vertical_plane_homography(K, R_c_wrt_l', -R_c_wrt_l'* t_c_wrt_l);
H_right = vertical_plane_homography(K, R_c_wrt_r', -R_c_wrt_r'* t_c_wrt_r);
T_left = projective2d(H_left');
T_right = projective2d(H_right');
rec_left = imwarp(im, T_left);
rec_right = imwarp(im, T_right);
figure, imshow (rec_left);
title('reconstruction of the left vertical face');
figure, imshow(rec_right);
title('reconstruction of the right vertical face');

% the reconstruction matrices obtained in the previous steps are a mapping
% from image points on the verical faces to real points, so we can use them
% to perform localization of points in the real world
% extract interesting points
corners = detectHarrisFeatures(rgb2gray(im), 'MinQuality',0.02, ...
    'FilterSize', 45);
[features, validCorners] = extractFeatures(rgb2gray(im), corners);
pts_l = points_selection(im, validCorners, "select " + ...
    "points on the left vertical face");
pts_r = points_selection(im, validCorners, "select " + ...
    "points on the right vertical face");
% bring the left points to the real world
vl_pts = pts_l * H_left';
vl_pts = vl_pts ./ vl_pts(:, 3);
% now they are in a reference system with xy plane on the left face, 
% express them in O_l
vl_pts = [vl_pts(:, 1), zeros(size(vl_pts, 1), 1), vl_pts(:, 2)];
% bring the right points to the real world
vr_pts = pts_r * H_right';
vr_pts = vr_pts ./ vr_pts(:, 3);
% now they are in a reference system with xy plane on the right face, 
% express them in O_r
vr_pts = [vr_pts(:, 1), zeros(size(vr_pts, 1), 1), vr_pts(:, 2)];
% rototraslate them, obtaining their expression in O_r
vr_pts = vr_pts * R_r_wrt_l' + t_r_wrt_l';
figure();
pcshowpair(pointCloud(left_face), pointCloud(vl_pts), 'MarkerSize', 200);
hold on
pcshowpair(pointCloud(right_face), pointCloud(vr_pts), 'MarkerSize', 200);


