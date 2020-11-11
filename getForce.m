function [force] = getForce(ipl,dynPress,alpha,beta,params,controls)
%% Compute forces in body (vehicle) frame
% alpha is in radians
mass = params.mass;
Sref = params.Sref; %square feet
g = params.g; % ft/sec squared
ThrustMag = params.Thrust; %lb
phi_T = params.phi_T; % angle between thrust and x-axis in degrees
CL0 = params.CL0;
CLalpha = params.CLalpha; %per radian
CLdelta_e = params.CLdelta_e;
CydeltaR = params.CydeltaR;
CyBeta = params.CyBeta;
delta_r = controls.delta_r;
delta_e = controls.delta_e;
%% Compute lift
C_lift = CL0 + CLalpha*alpha + CLdelta_e*delta_e;
Lift_stab = dynPress*Sref*C_lift; % in stability frame
%% Compute drag
CD = params.CD0 + C_lift^2/(pi*params.AR*params.OSE);
Drag_stab = dynPress*Sref*CD; % in stability frame
%% Compute side force
Cy = CyBeta*beta + CydeltaR*delta_r;
sideForce = Cy*dynPress*Sref;
%% Compute weight in aircraft frame
[transform] = transformItoV(ipl);
weight = mass*transform*[0; 0; g]; % weight vector in vehicle frame
%% Compute thrust
Thrust = ThrustMag*[cosd(phi_T); 0; -sind(phi_T)]; % Thrust in vehicle frame in pounds
%% Transform forces into the vehicle frame
force = [Lift_stab*sin(alpha)-Drag_stab*cos(alpha); sideForce; -Lift_stab*cos(alpha)-Drag_stab*sin(alpha)]+weight+Thrust;
end