function A = triad(b1, r1, b2, r2)
    
    % Ensure column vectors
    b1 = b1(:);
    r1 = r1(:);
    b2 = b2(:);
    r2 = r2(:);


    % Builds two right-handed triads
    v1 = r1;
    
    rx = cross(r1, r2);
    v2 = rx / norm(rx);

    v3 = cross(v1, v2);

    w1 = b1;
    
    bx = cross(b1, b2);
    w2 = bx / norm(bx);

    w3 = cross(w1, w2);

    % Computes attitude matrix
    A = [w1, w2, w3] * [v1, v2, v3]';
end