function vp = point_ls (h_lines)
    % for the duality principle, fitting the intersection point of multiple
    % lines is the same as fitting a line through multiple points
    vp = line_ls(h_lines);
end