function W = skew_mat(vec)    
% ensure column vector
    vec = vec(:);

    % skew-symmetric cross-product matrix
    vec_cross = [  0       -vec(3)   vec(2);
                  vec(3)     0        -vec(1);
                 -vec(2)   vec(1)     0      ];
    W = vec_cross;