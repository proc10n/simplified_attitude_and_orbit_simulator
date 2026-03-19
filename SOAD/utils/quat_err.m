function q_err = quat_err(q1, q2)
% QUAT_ERR  Error quaternion between two attitudes.
%
%   q_err = quat_err(q1, q2)
%
%   Computes q_err = q2^{-1} ⊗ q1, representing the rotation from q2 to q1
%   expressed in the body frame of q2. Scalar-first convention [q0; q1; q2; q3].
%
%   Enforces shortest-path (q_err(1) >= 0).
%
%   Inputs:
%       q1  - 4x1 unit quaternion (target)
%       q2  - 4x1 unit quaternion (reference/estimate)
%
%   Output:
%       q_err - 4x1 unit error quaternion

    q1 = q1(:);
    q2 = q2(:);

    q2_conj = [q2(1); -q2(2:4)];

    q_err = quat_mul(q2_conj, q1);

    if q_err(1) < 0
        q_err = -q_err;
    end
end