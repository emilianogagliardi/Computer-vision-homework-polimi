function selected = select_lines_from_click(lines, image, title)
    % takes in input an array containing lines in the form returned by the
    % matlab hough space algorithm, plot them, and ask to the user to
    % select a subset of them by clicking on the image
    fig = plot_lines(image, lines, 'red', title);
    [x, y] = getpts();
    selected = [];
    for ii = 1:length(x)
        pt = [x(ii), y(ii)];
        min_d = inf;
        nearest = lines(1);
        for jj = 1:length(lines)
            d = distance_segment_point(lines(jj), pt);
            if d < min_d
                min_d = d;
                nearest = lines(jj);
            end
        end
        selected = [selected nearest];
    end
    close(fig);
end

function d = distance_segment_point(line, pt)
    nearest = project_point_to_line_segment(line.point1, line.point2, pt);
    d = sqrt((pt(1)-nearest(1))^2 + (pt(2)-nearest(2))^2);
end
   
function [q] = project_point_to_line_segment(A,B,p)
    % returns q the closest point to p on the line segment from A to B 
    % vector from A to B
    AB = (B-A);
    % squared distance from A to B
    AB_squared = dot(AB,AB);
    if(AB_squared == 0)
        % A and B are the same point
        q = A;
    else
        % vector from A to p
        Ap = (p-A);
        % from http://stackoverflow.com/questions/849211/
        % Consider the line extending the segment, parameterized as A + t 
        % (B - A). We find projection of point p onto the line. 
        % It falls where t = [(p-A) . (B-A)] / |B-A|^2
        t = dot(Ap,AB)/AB_squared;
        if (t < 0.0) 
            % "Before" A on the line, just return A
            q = A;
        else
            if (t > 1.0)
                % "After" B on the line, just return B
                q = B;
            else
                % projection lines "inbetween" A and B on the line
                q = A + t * AB;
            end
        end
    end
end