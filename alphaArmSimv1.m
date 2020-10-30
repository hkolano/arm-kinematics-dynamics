% dhparams in order: [a alpha d theta]
% lengths in mm
theta_a = atan2(145.3, 40)
% dhparams = [20   	pi/2	46.2    pi;
%            150.71	pi      0       -theta_a;
%             20      -pi/2	0   	-theta_a;
%             0   	pi/2	-180	pi/2;
%             0       0       0   	-pi/2];

% Set up robot with DH Parameters
BaseL = Revolute('a',   20,     'alpha', pi/2,  'd',    46.2,   'offset', pi,       'qlim', [-175*pi/180, 175*pi/180]);
Link1 = Revolute('a',   150.71, 'alpha', pi,    'd',    0,      'offset', -theta_a, 'qlim', [0, 200*pi/180]);
Link2 = Revolute('a',   20,     'alpha', -pi/2, 'd',    0,      'offset', -theta_a, 'qlim', [0, 200*pi/180]);
Link3 = Revolute('a',   0,      'alpha', pi/2,  'd',    -180,   'offset', pi/2,     'qlim', [-175*pi/180, 175*pi/180]);
Link4 = Revolute('a',   0,      'alpha', 0,     'd',    0,      'offset', -pi/2,    'qlim', [0, pi/2]);


Qdegrees= [0 0 0 0 0];%[20, 90, 20, 45, 10];
Qspace = [0 0 0 0 0];

% Convert joint space from degrees to radians
for i = 1:5
    Qspace(i) = Qdegrees(i)*pi/180;
end

% make serial link object
alphaArm = BaseL + Link1 + Link2 + Link3 + Link4
% show the arm
alphaArm.teach()
% meshpath = 'C:/Users/hkolano/Stuff/OSU/Research/2020_Fall/Modeling_Code/AlphaMeshes'
% alphaArm.plot3d(Qspace)%, 'path', meshpath)
% Calculate the end effector position
Talpha_end = alphaArm.fkine(Qspace)