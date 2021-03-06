%{
Some shenanigans with dynamics on the Alpha arm.
Last modified by Hannah Kolano 2/19/2021

Current assumptions:
Motors are frictionless
No drag
%}

% clf;
addpath('C:\Users\hkolano\Documents\GitHub\ModernRobotics\packages\MATLAB\mr')
close all

%% Import the arm setup
alphaArm = alphaSetup();
[a_joint_frames, a_link_frames, MlistForward, MlistBackward, Slist, Alist, Glist, GaPlist] = AlphaKinematics();
% alpha_joint_frames = [Tnaught, T_0e, T_0d, T_0c, T_0b, T_0a];
% alpha_link_frames = [Tnaught, T_0_L1, T_0_L2, T_0_L3, T_0_L4, T_0_ee];
% M(i) = M_(i-1)_i  where i = link frame # // M(1) = M_0_1, M(2) = M_1_2

% homog T of end effector in the home configuration
M_home = [-1 0 0 -.3507; 0 1 0 0; 0 0 -1 0.0262; 0 0 0 1];

%  theta_limits = [-175, 175; ...
%         -74.61, 125.89; ...
%         -164.61, 35.39; ...
%         -165, 165; ...
%         -180, 180]*pi/180;


%% ---------- Dynamics ----------
g = [0; 0; 9.807]; % in m/s2
% thetalist = [0 0 0 0 0].';
thetalist = [0 0 0 0 0].'*pi/180;
dthetalist = [0; 0; 0; 0; 0];
ddthetalist = [0; 0; 0; 0; 0];
Ftip = [0; 0; 0; 0; 0; 0];

% T_screws = FKinSpace(M_home, Slist, thetalist);

positions = thetalist;
velocities = dthetalist;
accelerations = ddthetalist;
% torques = [0; 0; 0; 0; 0];

curr_theta = thetalist; curr_dtheta = dthetalist; curr_ddtheta = ddthetalist;
[MassM, RHS, taulist] = closedFormInverseDynamics(5, thetalist, dthetalist, ddthetalist, Ftip, g);
torques = taulist;
theta_start = curr_theta;
dtheta_start = curr_dtheta;
% theta_end = [-84 24 -96 120 0].'*pi/180;
theta_end = [0 0 0 0 0].';
dtheta_end = [0 0 0 0 0].'*pi/180;

iterations = 50;
dt = 0.1;
T = dt*iterations;
th1mth2 = theta_start - theta_end;
for i = 1:iterations
    t = dt*i;
    curr_ddtheta = 6*-2*(5*th1mth2+3*T*dtheta_start+2*T*dtheta_end)*t/T^3 + ...
        12*(15*th1mth2+8*T*dtheta_start + 7*T*dtheta_end)*t^2/T^4 + ...
        20*-3*(2*th1mth2+ T*dtheta_start + T*dtheta_end)*t^3/T^5;
    [theta_new, dtheta_new, ddtheta_new, taulist] = step_dynamics_forward(curr_theta, curr_dtheta, curr_ddtheta, Ftip, g, dt);
    curr_theta = theta_new; curr_dtheta = dtheta_new; %curr_ddtheta = ddtheta_new;'
    torques = [torques taulist];
    positions = [positions curr_theta];
    velocities = [velocities curr_dtheta];
    accelerations = [accelerations curr_ddtheta];
end


%% ---------- Plotting ----------
% Show the arm graphically
% alphaArm.plot(positions.', 'jointdiam', 1.5, 'nojvec', 'nobase', 'trail', {'r', 'LineWidth', 2}, 'noshadow');
% title('Alpha Arm over Sample Trajectory')
% xlabel('X (m)')
% ylabel('Y (m)')
% zlabel('Z (m)')
% hold on

% plot the base in the correct orientation
% [X, Y, Z] = cylinder(.020);
% surf(Z*.25, Y, X, 'FaceColor', 'k');

% figure 
% plot(0:dt:dt*iterations, positions(1,:))
% hold on
% plot(0:dt:dt*iterations, positions(2,:))
% plot(0:dt:dt*iterations, positions(3,:))
% plot(0:dt:dt*iterations, positions(4,:))
% xlabel('Time (s)')
% ylabel('Joint Angle (rad)') 
% legend('Base Joint', 'Shoulder Joint', 'Elbow Joint', 'Wrist Joint')
% title('Joint Angles over Trajectory')
% 
% figure 
% plot(0:dt:dt*iterations, velocities(1,:))
% hold on
% plot(0:dt:dt*iterations, velocities(2,:))
% plot(0:dt:dt*iterations, velocities(3,:))
% plot(0:dt:dt*iterations, velocities(4,:))
% xlabel('Time (s)')
% ylabel('Joint Velocities (rad/s)') 
% legend('Base Joint', 'Shoulder Joint', 'Elbow Joint', 'Wrist Joint')
% title('Joint Velocities over Trajectory')
% 
% figure 
% plot(0:dt:dt*iterations, accelerations(1,:))
% hold on
% plot(0:dt:dt*iterations, accelerations(2,:))
% plot(0:dt:dt*iterations, accelerations(3,:))
% plot(0:dt:dt*iterations, accelerations(4,:))
% xlabel('Time (s)')
% ylabel('Joint Accelerations (rad/s^2)') 
% legend('Base Joint', 'Shoulder Joint', 'Elbow Joint', 'Wrist Joint')
% title('Joint Accelerations over Trajectory')
% 
% figure 
% plot(0:dt:dt*iterations, torques(1,:))
% hold on
% plot(0:dt:dt*iterations, torques(2,:))
% plot(0:dt:dt*iterations, torques(3,:))
% plot(0:dt:dt*iterations, torques(4,:))
% xlabel('Time (s)')
% ylabel('Joint Torque (Nm)') 
% % legend('Joint E', 'Joint D', 'Joint C', 'Joint B')
% legend('Base Joint', 'Shoulder Joint', 'Elbow Joint', 'Wrist Joint')
% title('Joint Torques over Sample Trajectory')

% plot other coordinate frames
% trplot(T_screws, 'length', 0.2, 'thick', .2, 'rviz')
% trplot(a_link_frames(7).T, 'length', 0.15, 'thick', .75, 'rviz')
% trplot(T_0_e, 'length', 0.2, 'thick', 1, 'rviz')
% for i = 1:length(a_link_frames)
%     trplot(a_link_frames(i).T, 'length', .2, 'thick', 1, 'rviz', 'frame', '0');
% end

%% ---------- Dynamics iterator ---------
function [theta_new, dtheta_new, ddtheta_new, taulist] = step_dynamics_forward(thetalist, dthetalist, ddthetalist, Ftip, g, dt)
[MassM, RHS, taulist] = closedFormInverseDynamics(5, thetalist, dthetalist, ddthetalist, Ftip, g)
dqdt = dthetalist;
dqdot_dt = MassM\(taulist - RHS);

theta_new = thetalist + dqdt*dt;
dtheta_new = dthetalist + dqdot_dt*dt;
ddtheta_new = dqdot_dt;

end


