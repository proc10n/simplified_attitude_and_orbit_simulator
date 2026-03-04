%% ---- Gregorian -> Julian Date ----------------------------------------
function JD_UTC = greg2jd(utc0)
Y  = utc0(1);  Mo = utc0(2);  D  = utc0(3);
h  = utc0(4);  mi = utc0(5);  s  = utc0(6);

if Mo <= 2
    Y  = Y  - 1;
    Mo = Mo + 12;
end

A  = floor(Y / 100);
B  = 2 - A + floor(A / 4);

JD_UTC = floor(365.25 * (Y + 4716)) ...
       + floor(30.6001 * (Mo + 1)) ...
       + D + B - 1524.5 ...
       + (h + mi/60 + s/3600) / 24;
end