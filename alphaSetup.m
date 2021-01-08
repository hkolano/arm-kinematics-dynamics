%{
Set up the kinematic and dynamic properties of the Alpha arm. 
Last modified by Hannah Kolano 1/5/2021

Current assumptions:
In air
Motors are frictionless
No added mass due to being in water
No drag
Weight of end effector not included
%}

function alphaArm = alphaSetup()

    clf;

    %% Kinematics
    theta_a = atan2(145.3, 40);
    % dhparams in order: [a alpha d theta]
    % dhparams = [20   	pi/2	46.2    pi;
    %            150.71	pi      0       -theta_a;
    %             20      -pi/2	0   	-theta_a;
    %             0   	pi/2	-180	pi/2;
    %             0       0       0   	-pi/2];


    % Set up robot with DH Parameters
    % in mm
    Base = Revolute('a',   .020,     'alpha', pi/2,  'd',    .0462,   'offset', pi,       'qlim', [-175*pi/180, 175*pi/180]);
    Link1 = Revolute('a',   .15071, 'alpha', pi,    'd',    0,      'offset', -theta_a, 'qlim', [0, 200*pi/180]);
    Link2 = Revolute('a',   .020,     'alpha', -pi/2, 'd',    0,      'offset', -theta_a, 'qlim', [0, 200*pi/180]);
    Link3 = Revolute('a',   0,      'alpha', pi/2,  'd',    -.180,   'offset', pi/2,     'qlim', [-175*pi/180, 175*pi/180]);
    Link4 = Revolute('a',   0,      'alpha', 0,     'd',    0,      'offset', -pi/2,    'qlim', [0, pi/2]);

    %% Dynamics

    % masses in kg
    Base.m = .341;
    Link1.m = .194;
    Link2.m = .429;
    Link3.m = .115;
    Link4.m = .333;

    % center of mass location wrt link frame 
    % in m
    Base.r = [-75, -6, -3]/1000.0;
    Link1.r = [5, -1, 16]/1000.0;
    Link2.r = [73, 0, 0]/1000.0;
    Link3.r = [17 -26 -2]/1000.0;
    Link4.r = [0 3 -98]/1000.0;

    % Mass Moment of inertia (kg m^2)
    Base.I = [99 139 115; 139 2920 3; 115 3 2934]/1000.0/1000.0;
    Link1.I = [189 5 54; 5 213 3; 54 3 67]/1000.0/1000.0;
    Link2.I = [87 -76 -10; -76 3190 0; -10 0 3213]/1000.0/1000.0;
    Link3.I = [120 -61 -1; -61 62 0; -1 0 156]/1000.0/1000.0;
    Link4.I = [3709 2 -4; 2 3734 0; -4 0 79]/1000.0/1000.0;
    

    %% make serial link object
    alphaArm = Base + Link1 + Link2 + Link3 + Link4;
    alphaArm.name = 'Alpha';
    
end
