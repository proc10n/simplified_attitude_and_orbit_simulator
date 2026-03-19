function C = quat2dcm(q)
% Normalize the quaternion
q = q / norm(q);

% Extract the components of the quaternion
s = q(1);
i = q(2);
j = q(3);
k = q(4);

% Compute the direction cosine matrix
C = [s^2 + i^2 - j^2 - k^2,  2*(i*j + s*k), 2*(i*k - s*j);
     2*(i*j - s*k), s^2 - i^2 + j^2 - k^2,  2*(j*k + s*i);
     2*(i*k + s*j), 2*(j*k - s*i), s^2 - i^2 - j^2 + k^2];

end