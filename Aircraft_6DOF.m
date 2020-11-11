%% Aircraft 6DOF
%% Aircraft: T-37 Dragonfly
%% Define simulation control parameters
clear all;
t = 0; %initial time
prompt = input('Press 1 for longitudinal or 2 for lateral');
if prompt == 1
    maxTime = 600; % estimated maximum flight time in sec
elseif prompt == 2
    maxTime = 100;
end
dt = 0.05 ; % integration interval in sec
maxNum = maxTime/dt; %maximum number of steps in integration loop
%% Physical constants
params.g = 32.174; %ft/sec^2
%% Aircraft Parameters
params.AltCmd = 30000; %feet
params.SpdCmd = 456; %ft/sec
params.thetea0 = 0.0341; %Estimated initial Euler angle pitch angle in rad.
params.Ixx = 8000.; %slug-feet squared
params.Iyy = 3330.; %slug-feet squared
params.Izz = 11200.; %slug-feet squared
params.Ixz = -112.; %slug-feet squared
params.mass = 197.67; %slugs
params.wingSpan = 33.8; % feet
params.cbar = 5.47; %feet
params.Sref = 182; %ft^2
params.AR = params.wingSpan^2/params.Sref;
params.OSE = 1; %Oswald span efficiency
params.CLalpha = 5.184; %per radian
params.CL0 = 0.2 ;
params.CLdelta_e = 0.5; % per radian
params.CD0 = 0.0217;
params.weight = params.mass*params.g;
params.Cm0 = 0.0244;%.6;
params.CmAlpha = -.7; %per radian
params.CmDelta_e = -1.12; % per radian
params.Cmq = -0.08947;
params.CmAlphaDot = -0.0417;
params.phi_T = 0;% thrust vector pitch angle in deg.
params.dihedral = 0*pi/180; %dihedral in radians
params.sweep = 0*pi/180; %wing sweep in radians
params.Cy0 = 0;
params.CyBeta = -0.346; %per radian
params.CydeltaR = 0.2; %per radian
params.Croll0 = 0;
params.Cyaw0 = 0;
params.CrollBeta = -0.0945;
params.CrollP = -0.4426;
params.CrollR = 0.0927;
params.CrollDeltaA = 0.1813;
params.CrollDeltaR = 0.015;
params.CnBeta = 0.111;
params.CnDeltaA = -0.039;
params.CnDeltaR = -0.036;
params.CnR = -0.1394;
params.CnP = -0.0244;
controls.delta_e_gain1 = 0;%-2e-03;
controls.delta_e_gain2 = 0;%-1e-03;
%% Create Integrated Parameter List (state vector)
% ipl(1) = x = x position in inertial frame
% ipl(2) = y = y position in inertial frame
% ipl(3) = z = z position in inertial frame
% ipl(4) = u = x velocity in vehicle frame
% ipl(5) = v = y velocity in vehicle frame
% ipl(6) = w = z velocity in vehicle frame
% ipl(7) = p = roll rate in vehicle frame
% ipl(8) = q = pitch rate in vehicle frame
% ipl(9) = r = yaw rate in vehicle frame
% ipl(10) = phi = roll Euler angle in inertial frame
% ipl(11) = theta = pitch Euler angle in inertial frame
% ipl(12) = psi = yaw Euler angle in inertial frame
%% Initial Conditions
ipl = [0; 0; -params.AltCmd; params.SpdCmd; 0; 0; 0; 0; 0; 0; params.thetea0; 0];
[alpha,Thrust,ipl,delta_e0] = getInit(ipl,params);
params.Thrust = Thrust;
controls.delta_e0 = delta_e0;
controls.delta_e = delta_e0;
controls.delta_a = 0;
controls.delta_r = 0;
xDot = dxdt(t,ipl,params,controls);% initial xDot = [velocity and acceleration]. dxdt is a function.
% Start recording data for output & plots
speed(1) = norm(ipl(4:6));% Magnitude of xVector(4) through xVector(6).
output_vector(1, :)= [t,ipl'];% output time, position, velocity, acceleration
altitude(1) = -ipl(3);
AOA(1) = atand(ipl(6)/ipl(4));
alphaDot(1) = 0;
sideSlip(1) = asind(ipl(5)/speed(1));
[transform] = transformItoV(ipl);
vI = transform'*ipl(4:6);
FPA(1) = atand(vI(3)/sqrt(vI(1)^2+vI(2)^2));
elevator(1) = controls.delta_e*180/pi;
rudder(1) = 0;
aileron(1) = 0;
flightParameters(1,:) = [t elevator(1) rudder(1) aileron(1) ipl(1:6)' AOA(1) ...
        sideSlip(1)  ipl(10:12)'];
%% Trajectory Computation Loop
for ind = 2:maxNum
    if prompt == 1
        [controls] = getControls1(t,ipl,params,controls,vI);
    elseif prompt == 2
        [controls] = getControls2(t,ipl,params,controls,vI);
    end
    [T,X]=ode45(@(t,ipl)dxdt(t,ipl,params,controls),[t t+dt],ipl);% x and time are values of xVector at t+dt
    t = T(end); % last element in the "time" vector
    ipl = X(end,:); % last row in the "x" array. state at time(end).
%     if t>= 10. && t<= 10.1
%         ipl(5) = ipl(5) + 10.0;
%     end
    xDot = dxdt(t,ipl',params,controls);
    output_vector(ind, :)= [t,ipl];% xDot(4:6) is acceleration
    speed(ind) = norm(ipl(4:6));% magnitude of velocity
    [rho,acousticSpeed] = getRhoBritish(ipl);
    machNumber(ind) = speed(ind)/acousticSpeed;
    vI = transform\ipl(4:6)';
    FPA(ind) = atand(vI(3)/sqrt(vI(1)^2+vI(2)^2));
    altitude(ind) = -ipl(3);
    AOA(ind) = atand(ipl(6)/ipl(4));%angle-of-attack in degrees for output
    alphaDot(ind) = (xDot(4)*ipl(6)-xDot(6)*ipl(4))/(ipl(4)^2+ipl(6)^2);
    sideSlip(ind) = asind(ipl(5)/speed(ind));%sideslip angle in degrees for output
    elevator(ind) = controls.delta_e*180/pi; % elevator deflection history in degrees
    rudder(ind) = controls.delta_r*180/pi; %rudder deflection in degrees
    aileron(ind) = controls.delta_a*180/pi; %aileron deflection in degrees
    monitor = [t sideSlip(ind)*180/pi controls.delta_r altitude(ind)];
    flightParameters(ind,:) = [t elevator(ind) rudder(ind) aileron(ind) ipl(1:6) AOA(ind) ...
        sideSlip(ind)  ipl(10:12)];
    disp(monitor);
end
csvwrite('trajectory.csv',output_vector);
csvwrite('flightParameters.csv',flightParameters);
%% Plot results

if prompt == 1
    figure(1)
    grid minor
    plot(output_vector(:,1),AOA)
    axis([18 36 -3 12])
    ylabel('Alpha (deg)')
    xlabel('Time (sec)')
    title('Alpha vs Time')

    figure(2)
    grid minor
    plot(output_vector(:,1),output_vector(:,12))
    axis([18 36 0 .25])
    ylabel('Theta (deg)')
    xlabel('Time (sec)')
    title('Theta vs Time')


    figure(3)
    grid minor
    plot(output_vector(:,1),output_vector(:,12))
    axis([0 600 -.05 .25])
    ylabel('Theta (deg)')
    xlabel('Time (sec)')
    title('Theta vs Time')


    figure(4)
    grid minor
    plot(output_vector(:,1),speed)
    axis([0 600 435 475])
    ylabel('Speed (fps)')
    xlabel('Time (sec)')
    title('Speed vs Time')
    
   
    
elseif prompt == 2
    phi_deg = output_vector(:,11)*(180/pi);
    psi_deg = output_vector(:,13)*(180/pi);
    figure(5)
    grid minor
    plot(output_vector(:,1),psi_deg)
    hold on
    plot(output_vector(:,1),phi_deg)
    hold off
    axis([0 100 -3 5])
    legend('Psi','Phi')
    ylabel('Psi & Phi (deg)')
    xlabel('Time (sec)')
    title('Psi & Phi vs Time')


    figure(6)
    grid minor
    plot(sideSlip,AOA)
    axis([-3 3 1.952 1.956])
    ylabel('Alpha (deg)')
    xlabel('Beta (deg)')
    title('Alpha vs Beta')
    
end





