% F = -Vpg

[~, a_link_frames, ~, ~, ~, ~, ~, ~] = AlphaKinematics();
% a_link_frames = [Tnaught, T_0_L1, T_0_L2, T_0_L3, T_0_L4, T_0_L5, T_0ee];

%% Joint configuration 
    thetalist = [0 -.8 -1.3 0 0].
    J_s = JacobianSpace(Slist, thetalist);
    T_end = FKinSpace(M_home, Slist, thetalist)*MlistForward(n+1);
    J_b = Adjoint(TransInv(T_end))*J_s;
    JTFtip = J_b.'*Ftip;

rho = 1.002; % kg/L, fresh water
g = 9.81;

a_link_frames(2)

Fb_L1 = -0.018*rho*g;
Fb_L2 = -0.203*rho*g;
Fb_L3 = -0.025*rho*g;
Fb_L4 = -0.155*rho*g;