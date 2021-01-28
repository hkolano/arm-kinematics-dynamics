function bravoArm = bravoSetup()
%BRAVOSETUP Summary of this function goes here
%{
Set up the kinematic and dynamic properties of the Alpha arm. 
Last modified by Hannah Kolano 1/28/2021

Current assumptions:
In air
Motors are frictionless
No added mass due to being in water
No drag
Weight of end effector not included
%}

%% Kinematics
th_a = atan2(5.2, 293.55);
             
BLink1 = Revolute('a',   .046,    'alpha', -pi/2, 'd',    .1074,   'offset', pi);%,          'qlim', [-175*pi/180, 175*pi/180]);
BLink2 = Revolute('a',   .2936,  'alpha', 0,     'd',    0.0,     'offset', -pi/2);%,   'qlim', [0, pi]);
BLink3 = Revolute('a',   -.0408, 'alpha', -pi/2, 'd',    0.0,     'offset', pi/2);%,  'qlim', [0, pi]);
BLink4 = Revolute('a',   .0408,  'alpha', -pi/2, 'd',    -.1600,  'offset', 0.0);%,         'qlim', [-175*pi/180, 175*pi/180]);
BLink5 = Revolute('a',   .0408,  'alpha', -pi/2, 'd',    0.0,     'offset', pi);%,         'qlim', [0, pi]);
BLink6 = Revolute('a',   0,      'alpha', pi/2,  'd',    -.2235,  'offset', 0.0);
BLink7 = Revolute('a',   .120,   'alpha', 0,     'd',    0,       'offset', -pi/2);

%% Dynamics
% link masses
BLink1.m = 1.55;
BLink2.m = 1.98;
BLink3.m = 1.14;
BLink4.m = 1.14;
BLink5.m = 1.03;
BLink6.m = 1.04;
BLink7.m = 0.47;

% COM frame
BLink1.r = [17 -7 57]/1000.0;
BLink2.r = [117 15 6]/1000.0;
BLink3.r = [22 -29 1]/1000.0;
BLink4.r = [18 6 -117]/1000.0;
BLink5.r = [20 -24 1]/1000.0;
BLink6.r = [0 0 -128]/1000.0;
BLink7.r = [28 -1 0]/1000.0;

% Inertial properties
% BLink0.I = [2108 182 -15; 182 2573 -21; -15 -21 3483]/1000/1000.0;
BLink1.I = [11442 -484 3405; -484 12980 -1265; 3405 -1265 3202]/1000/1000.0;
BLink2.I = [3960 4200 3204; 4200 69099 -24; 3204 -24 70450]/1000/1000.0;
BLink3.I = [3213 -1548 -31; -1548 2327 6; -31 6 4340]/1000/1000.0;
BLink4.I = [21232 330 -3738; 330 22252 -1278; -3738 -1278 2054]/1000/1000.0;
BLink5.I = [2430 -1144 -40; -1144 2026 11; -40 11 3330]/1000/1000.0;
BLink6.I = [22359 1 -19; 1 22363 15; -19 15 936]/1000/1000.0;
BLink7.I = [244 -12 0; -12 1130 1; 0 1 1178]/1000/1000.0;

bravoArm = BLink1 + BLink2 + BLink3 + BLink4 + BLink5 + BLink6 + BLink7;
bravoArm.name = 'Bravo';

end

