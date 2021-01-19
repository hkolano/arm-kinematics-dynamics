
function [alpha_joint_frames, alpha_link_frames, MlistForward, MlistBackward, Slist, Alist, Glist] = AlphaKinematics()

%% ---------- SETUP ----------
    alphaArm = alphaSetup();
    Link1 = alphaArm.links(1);
    Link2 = alphaArm.links(2);
    Link3 = alphaArm.links(3);
    Link4 = alphaArm.links(4);
    Link5 = alphaArm.links(5);
    in_meters = 1; % if == 1, outputs values in m, elsewise in mm
    
%% ---------- TWISTS ---------
    % Twists calculated by hand by Hannah 01/2021
    % Format: [w, v]
    Slist = [[0; 0; 1; 0; 0; 0], ...
        [0; 1; 0; -.0462; 0; -0.02], ...
        [0; -1; 0; .0462; 0; 0.1707], ...
        [1; 0; 0; 0; 0.0262; 0], ...
        [0; 0; -1; 0; -0.3507; 0]];

%% ---------- HOMOGENEOUS TRANSFORMS (JOINTS) ----------
    
    QspaceStraight = pi/180*[0 74.61 164.61 0 0];
    [~, all] = alphaArm.fkine(QspaceStraight);
    
    Tnaught = SE3();
    T_0e = SE3([[-1 0 0 0]; [0 -1 0 0]; [0 0 1 0]; [0 0 0 1]]);
    T_0d = SE3(all(1));
    T_0c = SE3(all(2));
    T_0b = SE3(all(3));
    T_0a = SE3(all(4));
    
    alpha_joint_frames = [Tnaught, T_0e, T_0d, T_0c, T_0b, T_0a];
    
%% ---------- HOMOGENEOUS TRANSFORMS (LINKS) ----------

R0 = rpy2r([0 0 0]);     
    T_link1_from_jointE = SE3(R0, Link1.r);
    T_0_L1 = SE3(T_0e.T*T_link1_from_jointE.T);
    
    T_link2_from_jointD = SE3(R0, Link2.r);
    T_0_L2 = SE3(T_0d.T*T_link2_from_jointD.T);
    
    T_link3_from_jointC = SE3(R0, Link3.r);
    T_0_L3 = SE3(T_0c.T*T_link3_from_jointC.T);
    
    T_link4_from_jointB = SE3(R0, Link4.r);
    T_0_L4 = SE3(T_0b.T*T_link4_from_jointB.T);
    
    T_link5_from_jointA = SE3(R0, Link5.r); % jaw1 x=-10, jaw2 x=-10; both y = -45
    T_0_L5 = SE3(T_0a.T*T_link5_from_jointA.T);
    
    jaw_disp = 10; % 10mm away from COM of hand
    if in_meters == 1
        jaw_disp = jaw_disp/1000.0;
    end
    
    T_ee_from_L5 = SE3(R0, [0, -jaw_disp, 0]);
    T_0_ee = SE3(T_0_L5.T*T_ee_from_L5.T);
    alpha_link_frames = [Tnaught, T_0_L1, T_0_L2, T_0_L3, T_0_L4, T_0_L5, T_0_ee];
   
    num_link_frames = length(alpha_link_frames);
    inv_a_link_frames = [];
    for i = 1:num_link_frames
        inv_a_link_frames = [inv_a_link_frames SE3(inv(alpha_link_frames(i).T))];
    end  
    
%% ---------- RELATIVE LINK FRAMES (M) ----------
    M_backward = [];
    M_forward = [];
    
    for j = 2:num_link_frames
        M_j_jminus1 = SE3(inv_a_link_frames(j).T*alpha_link_frames(j-1).T);
        M_backward = [M_backward M_j_jminus1];
        
        M_jminus1_j = SE3(inv_a_link_frames(j-1).T*alpha_link_frames(j).T);
        M_forward = [M_forward M_jminus1_j];
    end

    % M_0_L1, M_L1_L2, M_L2_L3, M_L3_L4, M_L4_L5, M_L5_ee
    MlistForward = cat(3, M_forward(1).T, M_forward(2).T, M_forward(3).T, M_forward(4).T, M_forward(5).T, M_forward(6).T);
    MlistBackward = cat(3, M_backward(1).T, M_backward(2).T, M_backward(3).T, M_backward(4).T, M_backward(5).T, M_backward(6).T);
    
%% ---------- TWISTS IN LINK FRAMES ----------
    Alist = [];
    for i = 1:5
        S_i = Slist(:,i);
        A_i = Ad(inv_a_link_frames(i+1))*S_i;
        Alist = [Alist A_i];
    end
    
    Alist
     
 
    %% ---------- Spatial Inertia Matrices Gi ----------
    Gi_matrices = {};
    
    for i = 1:5
        link = alphaArm.links(i);
        G_i = [link.I(1,:) 0 0 0; link.I(2,:) 0 0 0; link.I(3,:) 0 0 0; ...
            0 0 0 link.m 0 0; 0 0 0 0 link.m 0; 0 0 0 0 0 link.m;];
        Gi_matrices{i} = G_i;
    end
    
    Glist = cat(3, Gi_matrices{1}, Gi_matrices{2}, Gi_matrices{3}, Gi_matrices{4}, Gi_matrices{5});
end