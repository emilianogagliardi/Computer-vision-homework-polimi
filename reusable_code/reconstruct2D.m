function Hrec = reconstruct2D(perp_pairs)
    %
    % !!!!!!!!!!!!! 
    % the input lines are assumed to come from an affine
    % reconstructed image, refer to affine_rectification.m before
    % !!!!!!!!!!!!!
    %
    % perp_pairs is a list of structs with fields 'l' and 'm'. The value of
    % the two fields must be lines in homogeneous coordinates, correponding
    % to perpendicular lines belonging to the plane on which the 2D
    % reconstruction is applied. Lines are assumed to be row vectors
    
    % apply least square for fitting c'inf*
    % already affine rectified image, so the degenerate dual conic
    % containing the image of the circular points can be written as
    % C = [a b 0;
    %      b 1 0;  because the image is affine rectified!
    %      0 0 0];
    % if m and l are perpendicular in the scene:
    % a * m(1) * l(1) + b * (m(1) * l(2) + m(2) * l(1)) = -m(2)*l(2)
    % apply least square to this model to fit a and b
    X = zeros(length(perp_pairs), 2);
    t = zeros(length(perp_pairs), 1);
    for ii = 1:length(perp_pairs)
        m = perp_pairs(ii).m;
        l = perp_pairs(ii).l;
        X(ii, 1) = m(1) * l(1);
        X(ii, 2) = (m(1) * l(2) + m(2) * l(1));
        t (ii) = -m(2)*l(2);
    end
    w = (X'*X) \ (X'*t);
    C = [w(1), w(2), 0;
         w(2),    1, 0;
         0,       0, 0];

    % decompose C'inf*
    [U, S, ~] = svd(C);

    % postmultiply U by a factor that makes S to be equal to the matrix
    % representing the coninc of the circular points in the scene
    % Cinf* = [1, 0, 0;
    %          0, 1, 0;
    %          0, 0, 0]
    % to obtain a transformation that maps the image of Cinf* to Cinf*
    % the transformation from the horizontal plane of the real scene and the
    % reconstructed image will be a similarity
    H = U * diag([sqrt(S(1, 1)), sqrt(S(2, 2)), 1]);
    % reconstruct the image
    Hrec = inv(H);
    
end