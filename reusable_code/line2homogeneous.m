function h_lines = line2homogeneous (lines)
    % lines must be a list of structs with fields 'point1' and 'point2', as
    % the one returned by the matlab function houghlines
    h_lines = zeros(length(lines), 3);
    for ii = 1 : length (lines)
        pt1 = [lines(ii).point1(1), lines(ii).point1(2)];
        pt1 = [pt1(1), pt1(2), 1];
        pt2 = [lines(ii).point2(1), lines(ii).point2(2)];
        pt2 = [pt2(1), pt2(2), 1];
        h_lines (ii, :) = cross(pt1, pt2);
        h_lines (ii, :) = h_lines(ii, :) / h_lines(ii, 3);
    end
end