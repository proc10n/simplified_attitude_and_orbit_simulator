%% ---- Helper: DCM to quaternion (Shepperd's method) ----
function q = dcm2quat(C)
    tr = C(1,1) + C(2,2) + C(3,3);

    q0 = 0.0;
    q1 = 0.0;
    q2 = 0.0;
    q3 = 0.0;

    vals = [tr, C(1,1), C(2,2), C(3,3)];
    [~, idx] = max(vals);

    switch idx
        case 1
            q0 = 0.5 * sqrt(1 + tr);
            k = 0.25 / q0;
            q1 = (C(2,3) - C(3,2)) * k;
            q2 = (C(3,1) - C(1,3)) * k;
            q3 = (C(1,2) - C(2,1)) * k;
        case 2
            q1 = 0.5 * sqrt(1 + 2*C(1,1) - tr);
            k = 0.25 / q1;
            q0 = (C(2,3) - C(3,2)) * k;
            q2 = (C(1,2) + C(2,1)) * k;
            q3 = (C(3,1) + C(1,3)) * k;
        case 3
            q2 = 0.5 * sqrt(1 + 2*C(2,2) - tr);
            k = 0.25 / q2;
            q0 = (C(3,1) - C(1,3)) * k;
            q1 = (C(1,2) + C(2,1)) * k;
            q3 = (C(2,3) + C(3,2)) * k;
        case 4
            q3 = 0.5 * sqrt(1 + 2*C(3,3) - tr);
            k = 0.25 / q3;
            q0 = (C(1,2) - C(2,1)) * k;
            q1 = (C(3,1) + C(1,3)) * k;
            q2 = (C(2,3) + C(3,2)) * k;
    end

    q = [q0; q1; q2; q3];

    % Enforce positive scalar part for consistency
    if q(1) < 0
        q = -q;
    end
end