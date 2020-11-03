% dhparams in order: [a alpha d theta]
% lengths in mm
th_a = atan2(5.2, 293.55);
dhparamsB = [46.0   	pi/2	107.4       pi;
            293.6	0.0     0.0         -pi/2+th_a;
            40.8    -pi/2   0.0         -pi/2-th_a;
            40.8   	-pi/2	-160.0      0.0;
            40.8    -pi/2   0.0         0.0;
            0.0     pi/2    -223.5      0.0;
            120.0   0.0     0.0         -pi/2;];
             
BaseL = Revolute('a',   46,     'alpha', pi/2,  'd',    107.4,   'offset', pi,          'qlim', [-175*pi/180, 175*pi/180]);
BLink1 = Revolute('a',   293.6,  'alpha', 0,     'd',    0.0,     'offset', th_a, 'qlim', [-pi/2, pi/2]);
BLink2 = Revolute('a',   40.8,   'alpha', -pi/2, 'd',    0.0,     'offset', pi/2-th_a, 'qlim', [-pi, 0]);
BLink3 = Revolute('a',   40.8,   'alpha', -pi/2, 'd',    -160.0,  'offset', pi,        'qlim', [-175*pi/180, 175*pi/180]);
BLink4 = Revolute('a',   40.8,   'alpha', -pi/2, 'd',    0.0,     'offset', pi,        'qlim', [-pi, 0]);
BLink5 = Revolute('a',   0,      'alpha', pi/2,  'd',    -223.5,  'offset', 0.0);
BLink6 = Revolute('a',   120.0,  'alpha', 0,     'd',    0,       'offset', -pi/2);

Qdegrees= [0 0 0 0 0 0 0];%[45, 45, 45, 45, 45, 45, 45];
Qspace = [0 0 0 0 0 0 0];        

% Convert joint space from degrees to radians
for i = 1:7
    Qspace(i) = Qdegrees(i)*pi/180;
end
BaseL.A(0)   
bravoArm = BaseL + BLink1 + BLink2 + BLink3 + BLink4 + BLink5 + BLink6;
bravoArm.name = 'Bravo';
bravoArm.teach(Qspace, 'jointdiam', 1);
Tbravo_end = bravoArm.fkine(Qspace);