function H = affine_rectification(varargin)
    % Each element of the varargin input must be a group of parallel lines
    % in homogeneous coordinates. The lines in each group must be parallel
    % between themselves. The groups of lines must be on the plane in which
    % the reconstruction is done or on a parallel plane. 
    % The lines are assumed to be row vectors in homogeneous coordinates,
    % so each element in the varargin must be an nx3 matrix with n >= 2.
    
    lines_groups_number = nargin - 1;
    
    % for each group of lines fit the vanishing point
    vps = zeros(lines_groups_number, 3);
    for ii = 1:lines_groups_number
        % if the lines are only two cross, because total least square gives
        % problem with ill conditioned matrix
        if size(varargin{ii}, 1) == 2
            lines = varargin{ii};
            vps(ii, :) = cross(lines(1, :), lines(2, :));
        else % otherwise total least square
            vps(ii, :) = point_ls(varargin{ii});
        end
    end
    
    % fit the line at infinity
    linf = line_ls(vps);
    
    H = [1, 0, 0; 0, 1, 0; linf];
end