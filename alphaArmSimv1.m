% dhparams in order: [a alpha d theta]
% lengths in mm
clf;
%% Set up the links
theta_a = atan2(145.3, 40);
% dhparams = [20   	pi/2	46.2    pi;
%            150.71	pi      0       -theta_a;
%             20      -pi/2	0   	-theta_a;
%             0   	pi/2	-180	pi/2;
%             0       0       0   	-pi/2];

addpath('C:\Users\hkolano\Documents\GitHub\ModernRobotics\packages\MATLAB\mr')

% Set up robot with DH Parameters
BaseL = Revolute('a',   20,     'alpha', pi/2,  'd',    46.2,   'offset', pi,       'qlim', [-175*pi/180, 175*pi/180]);
Link1 = Revolute('a',   150.71, 'alpha', pi,    'd',    0,      'offset', -theta_a, 'qlim', [0, 200*pi/180]);
Link2 = Revolute('a',   20,     'alpha', -pi/2, 'd',    0,      'offset', -theta_a, 'qlim', [0, 200*pi/180]);
Link3 = Revolute('a',   0,      'alpha', pi/2,  'd',    -180,   'offset', pi/2,     'qlim', [-175*pi/180, 175*pi/180]);
Link4 = Revolute('a',   0,      'alpha', 0,     'd',    0,      'offset', -pi/2,    'qlim', [0, pi/2]);

%% make serial link object
alphaArm = BaseL + Link1 + Link2 + Link3 + Link4;
alphaArm.name = 'Alpha';

Qspace0 = zeros(1, 5);
M_home = alphaArm.fkine(Qspace0);

Qspace1 = pi/180*[0 10 0 0 0];
Qspace2 = pi/180*[20, 20, 45, 30, 10];

[TW, T0] = alphaArm.twists(Qspace0);
T_end = prod([TW.exp(Qspace2) T0])

% alphaArm.teach(Qspace0, 'jointdiam', 1.5, 'jvec', 'nojoints');
% hold on
% trplot(T_end, 'length', 100, 'thick', 1, 'rviz')

s_jacob = alphaArm.jacob0(Qspace1)
b_jacob = alphaArm.jacobe(Qspace1)
