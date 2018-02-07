function lines = extract_lines_floor(input_im) 
    % apply the canny algorithm for finding the edges image and then use
    % hough space to find lines. The parameters have been manually tuned
    % aiming at finding the edges of the horizontal faces and some long
    % lines on the vertical faces
    gray_image = rgb2gray(input_im);
    edges = edge(gray_image,'canny', [0.03 0.04]);
    [H,T,R] = hough(edges, 'RhoResolution',0.5, 'Theta', -80:0.5:80);
    P = houghpeaks(H, 30, 'threshold',ceil(0.2*max(H(:))), 'NHoodSize', [301, 171]);
    lines = houghlines(edges,T,R,P, 'FillGap',80 ,'MinLength', 800);
end
