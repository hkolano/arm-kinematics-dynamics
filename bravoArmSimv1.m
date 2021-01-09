clf
% dhparams in order: [a alpha d theta]
% lengths in mm
th_a = atan2(5.2, 293.55);
% dhparamsB = [46.0     %pi/2	107.4       pi;
%             293.6     0.0     0.0         -pi/2+th_a;
%             40.8      -pi/2   0.0         -pi/2-th_a;
%             40.8   	-pi/2	-160.0      0.0;
%             40.8      -pi/2   0.0         0.0;
%             0.0       pi/2    -223.5      0.0;
%             120.0     0.0     0.0         -pi/2;];
             
BaseL = Revolute('a',   46,     'alpha', -pi/2,  'd',    107.4,   'offset', pi,          'qlim', [-175*pi/180, 175*pi/180]);
BLink1 = Revolute('a',   293.6,  'alpha', 0,     'd',    0.0,     'offset', th_a-pi/2,   'qlim', [0, pi]);
BLink2 = Revolute('a',   40.8,   'alpha', -pi/2, 'd',    0.0,     'offset', -pi/2-th_a,  'qlim', [0, pi]);
BLink3 = Revolute('a',   40.8,   'alpha', -pi/2, 'd',    -160.0,  'offset', 0.0,         'qlim', [-175*pi/180, 175*pi/180]);
BLink4 = Revolute('a',   40.8,   'alpha', -pi/2, 'd',    0.0,     'offset', 0.0,         'qlim', [0, pi]);
BLink5 = Revolute('a',   0,      'alpha', pi/2,  'd',    -223.5,  'offset', 0.0);
BLink6 = Revolute('a',   120.0,  'alpha', 0,     'd',    0,       'offset', -pi/2);

bravoArm = BaseL + BLink1 + BLink2 + BLink3 + BLink4 + BLink5 + BLink6;
bravoArm.name = 'Bravo';

Qspace0 = zeros(1, 7);
M_home = bravoArm.fkine(Qspace0);

Qspace1 = pi/180*[0 10 0 0 0 0 0];
Qspace2 = pi/180*[20, 0, 20, 45, 10, 30, 10];

[TW, T0] = bravoArm.twists(Qspace0);
T_end = prod([TW.exp(Qspace2) T0])

bravoArm.teach(Qspace0, 'jointdiam', 1, 'jvec');
% hold on
% trplot(T_end, 'length', 100, 'thick', 1, 'rviz')

s_jacob = bravoArm.jacob0(Qspace1)
b_jacob = bravoArm.jacobe(Qspace1)
