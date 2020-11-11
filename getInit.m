function [alpha,T,ipl,delta_e0] = getInit(ipl,params)
%% Initialize
%%
Sref = params.Sref;
CLalpha = params.CLalpha; %per radian
CL0 = params.CL0;
CD0 = params.CD0;
weight = params.weight;
CmAlpha = params.CmAlpha;
Cm0 = params.Cm0;
CmDelta_e = params.CmDelta_e;
CLdelta_e = params.CLdelta_e;
speed = params.SpdCmd;
[rho,acousticSpeed] = getRhoBritish(ipl);
dynPress = 0.5*rho*speed^2;
alpha1 = weight/(dynPress*Sref*CLalpha)-CL0/CLalpha;
% Iterate to find alpha and delta_e0
for i = 1:100
    delta_e1 = (-Cm0 - CmAlpha*alpha1)/CmDelta_e;
    C_lift = CL0 + CLalpha*alpha1 + CLdelta_e*delta_e1;
    CD = params.CD0 + C_lift^2/(pi*params.AR*params.OSE);
    alpha = (weight/(dynPress*Sref)-CL0-CLdelta_e*delta_e1)/(CLalpha+CD);
    if abs(alpha-alpha1)<1e-10
        break
    end
    alpha1 = alpha;
end
alpha = alpha1;
delta_e0 = delta_e1;
ipl(4) = speed*cos(alpha);
ipl(6) = speed*sin(alpha);
ipl(11) = alpha; % Initial theta = alpha
CL = CL0 + CLalpha*alpha + CLdelta_e*delta_e0;
CD = CD0 + CL^2/(pi*params.AR*params.OSE);
Drag = CD*dynPress*Sref;
T = Drag/cos(alpha);
end

