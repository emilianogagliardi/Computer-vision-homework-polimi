function [R, t] = loc_cam_wrt_right_face(R_c_wrt_l, t_c_wrt_l, R_r_wrt_l, ...
    t_r_wrt_l)
    % localize the camera with respect to the right face, given the
    % rotation matrix of the camera in the left face reference frame and
    % the position of the camera in the left reference frame, plus the
    % rotation of the right face in the left face frame and position of the
    % right face i nthe left face frame.
    
    A_c_wrt_l = [R_c_wrt_l, t_c_wrt_l; 0 0 0 1];
    A_l_wrt_r = [R_r_wrt_l', -R_r_wrt_l'*t_r_wrt_l; 0 0 0 1];
    A = A_l_wrt_r * A_c_wrt_l;
    R = A(1:3, 1:3);
    t = A(1:3, 4);
end
