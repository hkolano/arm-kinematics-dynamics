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
[a_joint_frames, a_link_frames, MlistForward, MlistBackward, Glist, Slist, Alist] = urdfConstructionAlpha();
% alpha_joint_frames = [Tnaught, T_0e, T_0d, T_0c, T_0b, T_0a];
% alpha_link_frames = [Tnaught, T_0_L1, T_0_L2, T_0_L3, T_0_L4, T_0_ee];
% M(i) = M_(i-1)_i  where i = link frame # // M(1) = M_0_1, M(2) = M_1_2

% Example joint configurations
Qspace0 = zeros(1, 5); % home
Qspace1 = pi/180*[0 45 0 0 0];
Qspace2 = pi/180*[20, 20, 45, 30, 0];
curr_config = Qspace0;

% homog T of end effector in the home configuration
M_home = alphaArm.fkine(Qspace0);

% Calculate twists
% [TW, T0] = alphaArm.twists(Qspace0);
% Forward product of exponentials (home config)
% T_end = prod([TW.exp(Qspace1) T0]);

%% ---------- Dynamics ----------
g = [0; 0; -9.807]; % in m/s2
thetalist = Qspace0.';
dthetalist = [0; 0; 0; 0; 0];
ddthetalist = [0; 0; 0; 0; 0];
Ftip = [0; 0; 0; 0; 0; 0];

% MR Mass Matrix and inverse dynamics
M_MR = MassMatrix(thetalist, MlistForward, Glist, Slist)
% MRtaulist = InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, MlistForward, Glist, Slist)

% basicInverseDynamics output
basic_taulist = basicInverseDynamics(5, thetalist, dthetalist, ddthetalist, Ftip)

% Peter Corke mass matrix and inverse dynamics
M_PC = alphaArm.inertia(thetalist.')
tau_PC = alphaArm.rne(thetalist.', dthetalist.', ddthetalist.')

%% ---------- Plotting ----------

% Show the arm graphically
alphaArm.plot(Qspace0, 'jointdiam', 1.5, 'jvec', 'nobase');
hold on

% plot the base in the correct orientation
[X, Y, Z] = cylinder(.020);
surf(Z*.25, Y, X, 'FaceColor', 'k');

% plot other coordinate frames
% trplot(a_joint_frames(6).T, 'length', 0.15, 'thick', .75, 'rviz')
% trplot(a_link_frames(7).T, 'length', 0.15, 'thick', .75, 'rviz')
% trplot(T_end, 'length', 0.2, 'thick', 1, 'rviz')
% for i = 1:length(a_link_frames)
%     trplot(a_link_frames(i).T, 'length', 100, 'thick', 1, 'rviz', 'frame', '0');
% end

%% ---------- Jacobians ----------

% Calculate the jacobians at a configuration
s_jacob = alphaArm.jacob0(Qspace1);
b_jacob = alphaArm.jacobe(Qspace1);
