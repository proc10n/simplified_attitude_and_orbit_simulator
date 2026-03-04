% -------- ECEF -> ECI (from GMST) --------
function [vec_eci, Cdot_ECI_ECEF] = ecef2eci(vec_ecef, gmst_deg, omegaE)
vec_ecef = vec_ecef(:);

th = gmst_deg * (pi/180);
c  = cos(th);
s  = sin(th);

C_ECI_ECEF = [ c -s 0;
               s  c 0;
               0  0 1];

Cdot_ECI_ECEF = omegaE * [ -s -c 0;
                             c -s 0;
                             0  0 0 ];

vec_eci = C_ECI_ECEF * vec_ecef;
end

