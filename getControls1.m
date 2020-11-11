function [controls] = getControls1(t,ipl,params,controls,vI)
%%
% ipl(1) = x = x position in inertial frame
% ipl(2) = y = y position in inertial frame
% ipl(3) = z = z position in inertial frame
% ipl(4) = u = x velocity in vehicle frame
% ipl(5) = v = y velocity in vehicle frame
% ipl(6) = w = z velocity in vehicle frame
% ipl(7) = p = roll rate in vehicle frame
% ipl(8) = q = pitch rate in vehicle frame
% ipl(9) = r = yaw rate in vehicle frame
% ipl(10) = phi = roll angle in inertial frame
% ipl(11) = theta = pitch angle in inertial frame
% ipl(12) = psi = yaw angle in inertial frame
%%

controls.delta_e = controls.delta_e0 + controls.delta_e_gain1*(ipl(3) + params.AltCmd) + ...
     controls.delta_e_gain2*vI(3);
controls.delta_a = 0;
controls.delta_r = 0;
if t>=20 && t<=20.5
    % controls.delta_r = -10*(pi/180)*sin(0.1*2*pi*(t-20)); 
    % controls.delta_a = -6.246*pi/180;%-1e-00*ipl(9);
    controls.delta_e = -5*pi/180;
end
end