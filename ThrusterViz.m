%{
Visualizing the thruster configuration on the Seabotix vehicle

Last modified 3/16/21 by Hannah Kolano
%}
addpath('C:\Users\hkolano\Documents\GitHub\ModernRobotics\packages\MATLAB\mr')
close all
OGdisps = cell(6, 1);
COMdisps = cell(6, 1);
rotations = cell(6, 1);
HomogTs = cell(6, 1);

% With respect to Scott's actuator URDF, where 0,0,0 is 2cm below the
% geometric center (8 cm under the COM)
OGdisps{1} = [0 0.12 .175];
OGdisps{2} = [0 -.12 .175];
OGdisps{3} = [.2794 0.075 0];
OGdisps{4} = [.2794 -0.075 0];
OGdisps{5} = [-.2794 0.075 0];
OGdisps{6} = [-.2794 -0.075 0];

% With respect to Scott's actuator URDF, but the reference point moved up
% 8cm to match the COM
COMdisps{1} = [0 0.12 .175-.08];
COMdisps{2} = [0 -.12 .175-.08];
COMdisps{3} = [.2794 0.075 -.08];
COMdisps{4} = [.2794 -0.075 -.08];
COMdisps{5} = [-.2794 0.075 -.08];
COMdisps{6} = [-.2794 -0.075 -.08];

rotations{1} = [-pi/3, pi/3, -pi/2];
rotations{2} = [pi/3, pi/3, pi/2];
rotations{3} = [0 0 -pi/4];
rotations{4} = [0 0 pi/4];
rotations{5} = [0 0 5*pi/4];
rotations{6} = [0 0 -5*pi/4];

COM_frame = SE3();

frame_matrices = cell(6, 1);
figure;
hold on
for i = 1:6
    % transform RPY into rotation matrix
    RotMat = rpy2r(rotations{i});
    % Form SE3 object from rotation matrix and displacement
    frame_matrices{i} = SE3(RotMat, COMdisps{i});
    trplot(frame_matrices{i}.T, 'rviz', 'length', .1, 'axes', 'equal', 'framelabel', string(i), 'text_opts', {{'FontSize'}, {18}})    
%     frame_matrices{i}
end
plot3(0, 0, 0, 'k.', 'MarkerSize', 20)
% plot3([-.3, -.3], [-.2, .2], [-.01, -.01], 'g-')
axis equal
grid on
% trplot(frame_matrices{1}, 'rviz', 'length', .125)

