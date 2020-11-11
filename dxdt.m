function xDot = dxdt(t,ipl,params,controls) %% Equations of motion
%%
% ipl(1) = x = x position in inertial frame % ipl(2) = y = y position in inertial frame % ipl(3) = z = z position in inertial frame % ipl(4) = u = x velocity in vehicle frame
% ipl(5) = v = y velocity in vehicle frame
% ipl(6) = w = z velocity in vehicle frame
% ipl(7) = p = roll rate in vehicle frame
% ipl(8) = q = pitch rate in vehicle frame
% ipl(9) = r = yaw rate in vehicle frame
% ipl(10) = phi = roll angle in inertial frame
% ipl(11) = theta = pitch angle in inertial frame
% ipl(12) = psi = yaw angle in inertial frame
%% Compute xDot
mass = params.mass; %slugs
Ixx = params.Ixx;
Iyy = params.Iyy;
Izz = params.Izz;
Ixz = params.Ixz;
[rho,acousticSpeed] = getRhoBritish(ipl);
speed = norm(ipl(4:6));
dynPress = 0.5*rho*(speed^2);
alpha = atan(ipl(6)/ipl(4));
beta = asin(ipl(5)/speed);
[transform] = transformItoV(ipl);
vInertial = inv(transform)*ipl(4:6);
force = getForce(ipl,dynPress,alpha,beta,params,controls);

uDot = ipl(9)*ipl(5)-ipl(8)*ipl(6)+force(1)/mass;
vDot = ipl(7)*ipl(6)-ipl(9)*ipl(4)+force(2)/mass;
wDot = ipl(8)*ipl(4)-ipl(7)*ipl(5)+force(3)/mass;

alphaDot = (uDot*ipl(6)-wDot*ipl(4))/(ipl(4)^2+ipl(6)^2);
moment = getMoment(ipl,dynPress,alpha,alphaDot,beta,params,controls); 

pDot = (Izz*moment(1)+Ixz*moment(3)-(Ixz*(Iyy-Ixx-Izz)*ipl(7)+(Ixz^2+Izz*(Izz-Iyy))*ipl(9))*ipl(8))/(Ixx*Izz-Ixz^2);
qDot = (1/Iyy)*(moment(2)-(Ixx-Izz)*ipl(7)*ipl(9)-Ixz*(ipl(7)^2-ipl(9)^2));
rDot = (Ixz*moment(1)+Ixx*moment(3)-(Ixz*(Iyy-Ixx-Izz)*ipl(9)+(Ixz^2+Ixx*(Ixx-Iyy))*ipl(7))*ipl(8))/(Ixx*Izz-Ixz^2);

phiDot = ipl(7)+tan(ipl(11))*sin(ipl(10))*ipl(8)+tan(ipl(11))*cos(ipl(10))*ipl(9);
thetaDot = ipl(8)*cos(ipl(10)) - ipl(9)*sin(ipl(10));
psiDot = (1/cos(ipl(11)))*(ipl(7)*sin(ipl(10))+ipl(9)*cos(ipl(12)));
xDot = [vInertial;uDot;vDot;wDot;pDot;qDot;rDot;phiDot;thetaDot;psiDot]; 
end
