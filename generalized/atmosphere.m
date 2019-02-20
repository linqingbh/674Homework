function air = atmosphere(h)
%Atmosphere returns a list of pressure, tempurature, density, speed of
%sound, and viscosity. Atmosphere must be given in meters. NOTE: This is
%only accurate up to 47 km.

h = h./1000;

T_sl    = 288.15;          % K
P_sl    = 1.01325.*10.^5;  % Pa
g_sl    = 9.80665;         % m/s^2
R       = 287.053;         % N*m/(kg*K) - Ideal gass constant
gamma   = 1.4;             % Ratio of specific heats
beta    = 1.458.*10^-6;    % kg/(smK^0.5)
S       = 110.4;           % K


air.T = T_sl - 71.5 + 2.*log(1+exp(35.75-3.25.*h)+exp(-3+0.0003.*h.^3));
air.P = P_sl.*exp(-0.118.*h - 0.0015.*h.^2./(1 - 0.018.*h + 0.0011.*h.^2));
air.rho = air.P./(R.*air.T);
air.a = sqrt(gamma.*R.*air.T);
air.mu = beta.*air.T.^(3./2)./(air.T+S);


end