%{
Some shenanigans with dynamics on the Alpha arm.
Last modified by Hannah Kolano 1/12/2021

Definitely not getting appropriate tau values right now.

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
[a_joint_frames, a_link_frames, MlistForward, MlistBackward, Slist, Alist, Glist] = AlphaKinematics();
% alpha_joint_frames = [Tnaught, T_0e, T_0d, T_0c, T_0b, T_0a];
% alpha_link_frames = [Tnaught, T_0_L1, T_0_L2, T_0_L3, T_0_L4, T_0_ee];
% M(i) = M_(i-1)_i  where i = link frame # // M(1) = M_0_1, M(2) = M_1_2

% Example joint configurations
Qspace0 = zeros(1, 5); % home
Qspace1 = pi/180*[0 74.61 164.61 0 0];
Qspace2old = pi/180*[16 27 -35 82 -13];
Qspace2new = Qspace1 + Qspace2old;

% homog T of end effector in the home configuration
M_home = [-1 0 0 -.3507; 0 1 0 0; 0 0 -1 0.0262; 0 0 0 1];

T_screws = FKinSpace(M_home, Slist, Qspace0.');


%% ---------- Dynamics ----------
g = [0; 0; -9.807]; % in m/s2
thetalist = Qspace0.';
dthetalist = [0; 0; 0; 0; 0];
ddthetalist = [0; 0; 0; 0; 0];
Ftip = [0; 0; 0; 0; 0; 0];

% % MR Mass Matrix and inverse dynamics
MassMatrix_MR = MassMatrix(thetalist, MlistForward, Glist, Slist)
% MRtaulist = InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, MlistForward, Glist, Slist)
% 
% % basicInverseDynamics output algorithm
closedform_MassMatrix = closedFormInverseDynamics(5, thetalist, dthetalist, ddthetalist, Ftip, g)
% basic_taulist = basicInverseDynamics(5, thetalist, dthetalist, ddthetalist, Ftip)
% 
% % Peter Corke mass matrix and inverse dynamics
MassMatrix_PC = alphaArm.inertia(Qspace1)
% tau_PC = alphaArm.rne(thetalist.', dthetalist.', ddthetalist.')


%% ---------- Plotting ----------
% Show the arm graphically
alphaArm.teach(Qspace1, 'jointdiam', 1.5, 'jvec', 'nobase');
hold on

% plot the base in the correct orientation
[X, Y, Z] = cylinder(.020);
surf(Z*.25, Y, X, 'FaceColor', 'k');

% plot other coordinate frames
trplot(T_screws, 'length', 0.2, 'thick', .2, 'rviz')
% trplot(a_link_frames(7).T, 'length', 0.15, 'thick', .75, 'rviz')
% trplot(T_0_e, 'length', 0.2, 'thick', 1, 'rviz')
% for i = 1:length(a_joint_frames)
%     trplot(a_link_frames(7).T, 'length', .2, 'thick', 1, 'rviz', 'frame', '0');
% end

%% ---------- Jacobians ----------

% Calculate the jacobians at a configuration
s_jacob = alphaArm.jacob0(Qspace1);
b_jacob = alphaArm.jacobe(Qspace1);
