% dhparams in order: [a alpha d theta]
% lengths in mm
theta_a = atan2(145.3, 40);
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

% make serial link object
alphaArm = BaseL + Link1 + Link2 + Link3 + Link4;
% show the arm
alphaArm.name = 'Alpha';

Qdegrees= zeros(5);%[20, 20, 45, 30, 0];% 
Qspace1 = QspaceDegreestoRadians(Qdegrees);
Qspace2 = QspaceDegreestoRadians([20, 20, 45, 30, 0]);

% Find twists in this configuration
[TW, T0] = alphaArm.twists(Qspace1);
fk1 = prod([TW.exp(Qspace1) T0])
fk2 = prod([TW.exp(Qspace2) T0])

codeFK1 = alphaArm.fkine(Qspace1)
codeFK2 = alphaArm.fkine(Qspace2)

alphaArm.teach(Qspace1, 'jointdiam', 1.5, 'jvec', 'joints');


function Qrad = QspaceDegreestoRadians(Qdeg)
    Qrad = zeros(length(Qdeg));
    for i = 1:length(Qrad)
        Qrad(i) = Qdeg(i)*pi/180;
    end
end