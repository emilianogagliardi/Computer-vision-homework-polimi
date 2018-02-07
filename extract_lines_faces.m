function lines = extract_lines_faces(input_im)
    gray_image = rgb2gray(input_im);
    edges = edge(gray_image,'canny', [0.05 0.11]);
    [H,T,R] = hough(edges, 'RhoResolution',0.5, 'Theta', -80:0.1:80);
    P = houghpeaks(H,150,'threshold',ceil(0.3*max(H(:))), 'NHoodSize', [71 17]);
    lines = houghlines(edges,T,R,P, 'FillGap',45 ,'MinLength', 380);
end

