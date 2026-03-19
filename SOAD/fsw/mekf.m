function q = mekf(mag_b, mag_ref, sun_b, sun_ref, gyro, dt, q_st, ...
                  mag_valid, sun_valid, gyro_valid, st_valid, cfg)

    persistent q_est P Q R R_st B_est kf_init_flag
    persistent q_last P_last B_last
    persistent w_prev
    persistent st_counter N_st

    % ---------- Init ----------
    if isempty(kf_init_flag)
        [q_est, P, Q, R, R_st, B_est] = kf_init(cfg, dt);
        q_last = q_est;  P_last = P;  B_last = B_est;
        w_prev = [0;0;0];

        N_st = cfg.N_st;
        st_counter = 0;

        kf_init_flag = true;
    end

    % ---------- Gyro ----------
    if gyro_valid
        w_prev = gyro;
    end
    w_hat = w_prev - B_est;

    % =============================================
    %  Propagation
    % =============================================

    q0 = q_est(1);  qv = q_est(2:4);
    Xi = [-qv.';  q0*eye(3) + skew_mat(qv)];
    dq = 0.5 * Xi * w_hat;

    q_est = q_est + dq*dt;

    nq = norm(q_est);
    if ~isfinite(nq) || nq < 1e-12
        q_est = [1;0;0;0];
    else
        q_est = q_est / nq;
    end
    if dot(q_est, q_last) < 0
        q_est = -q_est;
    end

    % Error-state covariance propagation
    F = [-skew_mat(w_hat)  -eye(3);
          zeros(3,3)        zeros(3,3)];

    Phi = eye(6) + F*dt;

    P = Phi*P*Phi.' + Q;

    % =============================================
    %  Measurement availability
    % =============================================

    A = quat2dcm(q_est);

    a1 = zeros(3,1);  a2 = zeros(3,1);
    mag_ok = false;    sun_ok = false;

    if mag_valid
        a1 = A * mag_ref;
        mag_ok = (norm(a1) > 1e-12) && (norm(mag_b) > 1e-12);
    end

    if sun_valid
        a2 = A * sun_ref;
        sun_ok = (norm(a2) > 1e-12) && (norm(sun_b) > 1e-12);
    end

    % Star tracker: gated by validity flag AND internal rate counter
    st_counter = st_counter + 1;
    do_st_update = st_valid && (st_counter >= N_st);
    if do_st_update
        st_counter = 0;
    end

    have_vec_meas = (mag_ok || sun_ok);

    if ~have_vec_meas && ~do_st_update
        q = q_est;
        q_last = q_est;  P_last = P;  B_last = B_est;
        return;
    end

    % =============================================
    %  Vector measurement update (mag + sun)
    % =============================================

    if have_vec_meas
        H = zeros(6,6);
        h = [a1; a2];
        y = [mag_b; sun_b];

        if mag_ok
            H(1:3,1:3) = skew_mat(a1);
        else
            h(1:3) = 0;  y(1:3) = 0;
        end

        if sun_ok
            H(4:6,1:3) = skew_mat(a2);
        else
            h(4:6) = 0;  y(4:6) = 0;
        end

        S = H*P*H' + R;
        S = 0.5*(S + S.');

        if ~all(isfinite(S(:))) || rcond(S) < 1e-12
            lam = 1e-9*max(1, mean(diag(abs(S))));
            if ~isfinite(lam) || lam <= 0
                lam = 1e-6;
            end
            S = S + lam*eye(6);
        end

        K = (P*H') / S;

        innov = y - h;

        I_KH = eye(6) - K*H;
        P = I_KH*P*I_KH' + K*R*K';
        P = 0.5*(P + P.');

        dx     = K*innov;
        dalpha = dx(1:3);
        dbeta  = dx(4:6);

        q0u = q_est(1);  qvu = q_est(2:4);
        Xi_u = [-qvu.';  q0u*eye(3) + skew_mat(qvu)];
        q_est = q_est + 0.5*Xi_u*dalpha;

        nq = norm(q_est);
        if ~isfinite(nq) || nq < 1e-12
            q_est = q_last;  P = P_last + 1e-6*eye(6);
        else
            q_est = q_est / nq;
            if dot(q_est, q_last) < 0
                q_est = -q_est;
            end
            B_est = B_est + dbeta;

            E = eye(6);
            E(1:3,1:3) = eye(3) - 0.5*skew_mat(dalpha);
            P = E*P*E.';
            P = 0.5*(P + P.');
        end
    end

    % =============================================
    %  Star tracker update
    % =============================================

    if do_st_update
        q_e = quat_err(q_st, q_est);
        delta_alpha_meas = 2 * q_e(2:4);

        H_st = zeros(3,6);
        H_st(:,1:3) = eye(3);

        S_st = H_st * P * H_st.' + R_st;
        S_st = 0.5 * (S_st + S_st.');
        if ~all(isfinite(S_st(:))) || rcond(S_st) < 1e-12
            lam = 1e-9 * max(1, mean(diag(abs(S_st))));
            if ~isfinite(lam) || lam <= 0
                lam = 1e-6;
            end
            S_st = S_st + lam * eye(3);
        end
        K_st = (P * H_st.') / S_st;

        innov_st = delta_alpha_meas;

        I_KH = eye(6) - K_st * H_st;
        P = I_KH * P * I_KH.' + K_st * R_st * K_st.';
        P = 0.5 * (P + P.');

        dx     = K_st * innov_st;
        dalpha = dx(1:3);
        dbeta  = dx(4:6);

        q0u = q_est(1);  qvu = q_est(2:4);
        Xi_u = [-qvu.';  q0u*eye(3) + skew_mat(qvu)];
        q_est = q_est + 0.5 * Xi_u * dalpha;

        nq = norm(q_est);
        if ~isfinite(nq) || nq < 1e-12
            q_est = q_last;
            P = P_last + 1e-6 * eye(6);
        else
            q_est = q_est / nq;
            if dot(q_est, q_last) < 0
                q_est = -q_est;
            end
            B_est = B_est + dbeta;

            E = eye(6);
            E(1:3,1:3) = eye(3) - 0.5*skew_mat(dalpha);
            P = E * P * E.';
            P = 0.5 * (P + P.');
        end
    end

    % ---------- Final guard ----------
    if ~all(isfinite([q_est; diag(P); B_est]))
        q_est = q_last;  P = P_last;  B_est = B_last;
    else
        q_last = q_est;  P_last = P;  B_last = B_est;
    end

    q = q_est;
end


% ---- Filter-specific local functions ----

function [q0, P0, Q, R, R_st, B0] = kf_init(cfg, Ts)

    q0 = [1; 0; 0; 0];
    B0 = cfg.B0;

    P0 = diag([cfg.P0_att  * ones(1,3), ...
               cfg.P0_bias * ones(1,3)]);

    % Process noise                              % TODO: q_theta should be sigma_gyro^2 * Ts, not Ts^2
    q_theta = (cfg.sigma_gyro^2)    * Ts^2 * ones(1,3);
    q_bias  = (cfg.sigma_bias_rw^2) * Ts   * ones(1,3);

    Q = diag([q_theta q_bias]);

    % Measurement noise
    R_mag = (cfg.sigma_mag^2) * eye(3);
    R_sun = (cfg.sigma_sun^2) * eye(3);
    R     = blkdiag(R_mag, R_sun);

    R_st  = (cfg.sigma_st^2) * eye(3);
end