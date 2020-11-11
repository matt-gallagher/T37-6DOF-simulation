function [rho,acousticSpeed] = getRhoBritish(ipl)
%% Constants

To = 518.67; %temp at MSL
Po = 2116.23; %pressure at MSL
L = -0.003566; %lapse rate
g = 32.174; %gravitational acceleration
R = 1716.49; %J/kg-K
gam = 1.4; %specific heat ratio

%% Model

z = -ipl(3); %z is third term in position vector
T = To + L * (abs(z)); %Temp at current altitude
gLR = -(g/(L * R)); %Pressure equation exponent
P = Po * (T/To)^gLR; %Pressure at curernt altitude
rho = P/(R * T); %Density at current altitude
acousticSpeed = sqrt(gam * R * T); %Acoustic speed
end