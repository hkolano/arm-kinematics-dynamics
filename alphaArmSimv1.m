%{
Some shenanigans with dynamics on the Alpha arm.
Last modified by Hannah Kolano 2/5/2021

Current assumptions:
In air
Motors are frictionless
No added mass due to being in water
No drag
%}

% clf;
addpath('C:\Users\hkolano\Documents\GitHub\ModernRobotics\packages\MATLAB\mr')

%% Import the arm setup
alphaArm = alphaSetup();
[a_joint_frames, a_link_frames, MlistForward, MlistBackward, Slist, Alist, Glist, GaPlist] = AlphaKinematics();
% alpha_joint_frames = [Tnaught, T_0e, T_0d, T_0c, T_0b, T_0a];
% alpha_link_frames = [Tnaught, T_0_L1, T_0_L2, T_0_L3, T_0_L4, T_0_ee];
% M(i) = M_(i-1)_i  where i = link frame # // M(1) = M_0_1, M(2) = M_1_2

% Example joint configurations
Qspace0 = zeros(1, 5); % home
Qspace1 = pi/180*[0 0 0 0 0];
Qspace2old = pi/180*[16 27 -35 82 -13];
Qspace2new = Qspace1 + Qspace2old;

% homog T of end effector in the home configuration
M_home = [-1 0 0 -.3507; 0 1 0 0; 0 0 -1 0.0262; 0 0 0 1];


%% ---------- Dynamics ----------
g = [0; 0; 9.807]; % in m/s2
thetalist = [0 0 0 0 0].';
dthetalist = [0; 0; 0; 0; 0];
ddthetalist = [0; 1; 0; 0; 0];
Ftip = [0; 0; 0; 0; 0; 0];

T_screws = FKinSpace(M_home, Slist, thetalist);

% closedFormDynamics output 
 [MassM, RHS, taulist] = closedFormInverseDynamics(5, thetalist, dthetalist, ddthetalist, Ftip, g);
 new_ddthetalist = MassM\(taulist - RHS)


%% ---------- Plotting ----------
% Show the arm graphically
alphaArm.teach(thetalist.', 'jointdiam', 1.5, 'jvec', 'nobase');
hold on

% plot the base in the correct orientation
[X, Y, Z] = cylinder(.020);
surf(Z*.25, Y, X, 'FaceColor', 'k');

% plot other coordinate frames
% trplot(T_screws, 'length', 0.2, 'thick', .2, 'rviz')
% trplot(a_link_frames(7).T, 'length', 0.15, 'thick', .75, 'rviz')
% trplot(T_0_e, 'length', 0.2, 'thick', 1, 'rviz')
% for i = 1:length(a_link_frames)
%     trplot(a_link_frames(i).T, 'length', .2, 'thick', 1, 'rviz', 'frame', '0');
% end

