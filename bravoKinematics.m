function [bravo_joint_frames, bravo_link_frames, Slist] = bravoKinematics()
%BRAVOKINEMATICS Summary of this function goes here
%   Detailed explanation goes here

    bravoArm = bravoSetup();
    
%% ---------- TWISTS ----------
    % Twists calculated by hand by Hannah 01/2021, arm straight up
    % Format: [w, v]
    Slist = [[0; 0; 1;  0;      0;      0], ...
        [0; -1;   0;    .1074;  0;      .046], ...
        [0; -1;   0;    .401;   0;      .046], ...
        [0;  0;  -1;    0;     -.0052;  0], ...
        [0;  1;   0;   -.561;   0;     -.046], ...
        [0;  0;  -1;    0;     -.0052;  0], ...
        [0;  1;   0;   -.7845;  0;     -.0052]];

%% ---------- Homogeneous Transforms (joints) ----------
    QspaceStraight =[0 0 0 0 0 0 0];
    [~, all] = bravoArm.fkine(QspaceStraight);
    
    Tnaught = SE3();
    T_0g = Tnaught;
    T_0f = SE3(all(1))
    T_0e = SE3(all(2));
    T_0d = SE3(all(3));
    T_0c = SE3(all(4));
    T_0b = SE3(all(5));
    T_0a = SE3(all(6));
    T_0ee = SE3(all(7));
    
    bravo_joint_frames = [Tnaught, T_0g, T_0f, T_0e, T_0d, T_0c, T_0b, T_0a];
    
%% ---------- HOMOGENEOUS TRANSFORMS (LINKS) ----------

    R0 = rpy2r([0 0 0]);     
    T_link1_from_jointG = SE3(R0, bravoArm.links(1).r);
    T_0_L1 = SE3(T_0g.T*T_link1_from_jointG.T);
    
    % There's no way this one is right
    T_link2_from_jointF = SE3(R0, bravoArm.links(2).r);
    T_0_L2 = SE3(T_0f.T*T_link2_from_jointF.T);
    
    T_link3_from_jointE = SE3(R0, bravoArm.links(3).r);
    T_0_L3 = SE3(T_0e.T*T_link3_from_jointE.T);
    
    T_link4_from_jointD = SE3(R0, bravoArm.links(4).r);
    T_0_L4 = SE3(T_0d.T*T_link4_from_jointD.T);
    
    % This one is also sketchy
    T_link5_from_jointC = SE3(R0, bravoArm.links(5).r); 
    T_0_L5 = SE3(T_0c.T*T_link5_from_jointC.T);
    
    T_link6_from_jointB = SE3(R0, bravoArm.links(6).r);
    T_0_L6 = SE3(T_0b.T*T_link6_from_jointB.T);
    
    T_link7_from_jointA = SE3(R0, bravoArm.links(7).r);
    T_0_L7 = SE3(T_0a.T*T_link7_from_jointA.T);
%     
%     jaw_disp = 10/1000.0; % 10mm away from COM of hand
%     
%     T_ee_from_L5 = SE3(R0, [0, -jaw_disp, 0]);
%     T_0_ee = SE3(T_0_L5.T*T_ee_from_L5.T);
    bravo_link_frames = [Tnaught, T_0_L1, T_0_L2, T_0_L3, T_0_L4, T_0_L5, T_0_L6, T_0_L7];
   
%     num_link_frames = length(alpha_link_frames);
%     inv_a_link_frames = [];
%     for i = 1:num_link_frames
%         inv_a_link_frames = [inv_a_link_frames SE3(inv(alpha_link_frames(i).T))];
%     end  
    
    % inv_a_link_frames = [Tnaught, T_L1_0, T_L2_0, T_L3_0, T_L4_0, T_L5_0, T_ee_0]
end

