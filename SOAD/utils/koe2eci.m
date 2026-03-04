function [r_eci, v_eci] = koe2eci(a, e, i, RAAN, w, v, mu)
    % Semi-latus rectum
    p = a * (1 - e^2);

    % Radius magnitude
    r = p / (1 + e*cos(v));

    % State in perifocal frame (PQW)
    r_pqw = [r*cos(v); r*sin(v); 0];
    v_pqw = sqrt(mu/p) * [-sin(v); e + cos(v); 0];

    % Rotation PQW -> ECI: R3(RAAN)*R1(i)*R3(w)
    Q_pqw_to_eci = R3(RAAN) * R1(i) * R3(w);

    % ECI vectors
    r_eci = Q_pqw_to_eci * r_pqw;
    v_eci = Q_pqw_to_eci * v_pqw;
end