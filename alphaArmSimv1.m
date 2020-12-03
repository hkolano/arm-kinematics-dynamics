%{
Some shenanigans with dynamics on the Alpha arm.
Last modified by Hannah Kolano 12/2/2020

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
% Link frames: [ 0 1 2 3 4 jaws ]
% Joint frames: [ origin E D C B A ] (ie origin 0 1 2 3 4)
% M(i) = M_i_(i-1)  where i = link frame # // M(1) = M_1_0, M(2) = M_2_1

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
g = [0; 0; -9.8];
thetalist = Qspace0.';
dthetalist = [0; 0; 0; 0; 0];
ddthetalist = [0; 0; 0; 0; 0];
Ftip = [0; 0; 0; 0; 0; 0];
% M = MassMatrix(thetalist, Mlist, Glist, Slist)
taulist = InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist)

%% ---------- Plotting ----------

% Show the arm graphically
alphaArm.teach(Qspace0, 'jointdiam', 1.5, 'jvec', 'nobase');
hold on

% plot the base in the correct orientation
[X, Y, Z] = cylinder(20);
surf(Z*150, Y, X, 'FaceColor', 'k');

% plot other coordinate frames
% trplot(T_end, 'length', 100, 'thick', 1, 'rviz')
for i = 1:length(a_link_frames)
    trplot(a_link_frames(i).T, 'length', 100, 'thick', 1, 'rviz', 'frame', '0');
end
% trplot(a_link_frames(1).T, 'length', 100, 'thick', 1, 'rviz', 'frame', '0');
%% ---------- Jacobians ----------

% Calculate the jacobians at a configuration
s_jacob = alphaArm.jacob0(Qspace1);
b_jacob = alphaArm.jacobe(Qspace1);
