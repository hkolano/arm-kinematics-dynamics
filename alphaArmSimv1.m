%{
Some shenanigans with dynamics on the Alpha arm.
Last modified by Hannah Kolano 1/5/2021

Definitely not getting appropriate tau values right now.
Suspicion: links were labelled wrong. Confirm that M_i,j values are as
described in MR. 

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
[a_joint_frames, a_link_frames, Mlist, Glist, Slist] = urdfConstructionAlpha();
% alpha_joint_frames = [Tnaught, T_0e, T_0d, T_0c, T_0b, T_0a];
% alpha_link_frames = [Tnaught, T_0_L1, T_0_L2, T_0_L3, T_0_L4, T_0_ee];
% M(i) = M_(i-1)_i  where i = link frame # // M(1) = M_0_1, M(2) = M_1_2

% Example joint configurations
Qspace0 = zeros(1, 5); % home
Qspace1 = pi/180*[0 10 0 0 0];
Qspace2 = pi/180*[20, 20, 45, 30, 10];
curr_config = Qspace0;

% homog T of end effector in the home configuration
M_home = alphaArm.fkine(Qspace0);

% Calculate twists
[TW, T0] = alphaArm.twists(Qspace0);
% Forward product of exponentials (home config)
T_end = prod([TW.exp(Qspace2) T0]);

%% ---------- Dynamics ----------
Slist4dof = Slist(:, 1:4);
g = [0; 0; -9.807]; % in m/s2
thetalist = [0; 0; 0; 0];
dthetalist = [0; 0; 0; 0];
ddthetalist = [0; 0; 0; 0];
Ftip = [0; 0; 0; 0; 0; 0];
% M = MassMatrix(thetalist, Mlist, Glist, Slist)
taulist = InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist4dof)

%% ---------- Plotting ----------

% Show the arm graphically
alphaArm.teach(Qspace0, 'jointdiam', 1.5, 'jvec', 'nobase');
hold on

% plot the base in the correct orientation
[X, Y, Z] = cylinder(20);
surf(Z*150, Y, X, 'FaceColor', 'k');

% plot other coordinate frames
% trplot(T_end, 'length', 100, 'thick', 1, 'rviz')
% for i = 1:length(a_link_frames)
%     trplot(a_link_frames(i).T, 'length', 100, 'thick', 1, 'rviz', 'frame', '0');
% end

%% ---------- Jacobians ----------

% Calculate the jacobians at a configuration
s_jacob = alphaArm.jacob0(Qspace1);
b_jacob = alphaArm.jacobe(Qspace1);
