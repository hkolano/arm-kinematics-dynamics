% <joint name="${name}_joint5" type="revolute">
%           <parent link="${name}_base_link" />
%           <child link="${name}_shoulder_link" />
%           <origin xyz="0 0 0" rpy="0 0 3.141592" />
%           <axis xyz="0 0 1" />
%           <limit effort="9.0" lower="0.0" upper="6.2" velocity="0.5" />

function [alpha_joint_frames, alpha_link_frames, Mlist, Glist, Slist] = urdfConstructionAlpha()
    clf; 
    alphaArm = alphaSetup();
    BaseL = alphaArm.links(1);
    Link1 = alphaArm.links(2);
    Link2 = alphaArm.links(3);
    Link3 = alphaArm.links(4);
    Link4 = alphaArm.links(5);
    
    %% ---------- JOINT FRAMES ----------

    % Joint E frame wrt Base
    t5 = [0 0 0];
    rpy5 = [0 0 pi];
    R5 = rpy2r(rpy5);
    T_0e = SE3(R5, t5);

    %  <joint name="${name}_joint4" type="revolute">
    %           <parent link="${name}_shoulder_link" />
    %           <child link="${name}_upper_arm_link" />
    %           <origin xyz="0.020 0 0.046" rpy="1.57075 1.3 0" />
    %           <axis xyz="0 0 1" />
    %           <limit effort="9.0" lower="0.0" upper="3.5" velocity="0.5" />

    t4 = [20 0 46.2];
    rpy4 = [1.57075 1.302 0];
    R4 = rpy2r(rpy4);
    T_ed = SE3(R4, t4);

    %  <joint name="${name}_joint3" type="revolute">
    %           <parent link="${name}_upper_arm_link" />
    %           <child link="${name}_forearm_link" />
    %           <origin xyz="0.15 0 0" rpy="3.1415 0 1.3" />
    %           <axis xyz="0 0 1" />
    %           <limit effort="9.0" lower="0.0" upper="3.5" velocity="0.5" />

    t3 = [150.71 0 0];
    rpy3 = [pi 0 1.302];
    R3 = rpy2r(rpy3);
    T_dc = SE3(R3, t3);

    %  <joint name="${name}_joint2" type="revolute">
    %           <parent link="${name}_forearm_link" />
    %           <child link="${name}_wrist_link" />
    %           <origin xyz="0.020 0 0" rpy="-1.57075 -0.5 0" />
    %           <axis xyz="0 0 1" />
    %           <limit effort="5.0" lower="0.0" upper="6.0" velocity="1.0" />
    %       </joint>

    t2 = [20 0 0];
    rpy2 = [-1.57075 -1.57075 0]; % was -1.57075 -0.5 0
    R2 = rpy2r(rpy2);
    T_cb = SE3(R2, t2);


    %       <joint name="${name}_joint_jaw" type="fixed">
    %           <parent link="${name}_wrist_link" />
    %           <child link="${name}_jaw" />
    %           <origin xyz="0 0 -0.190" rpy="0 0 1.5707" />
    %       </joint>

    t1 = [0 0 -180]; % was 0 0 -190
    rpy1 = [0 1.5707 1.5707]; % was 0 0 1.5707
    R1 = rpy2r(rpy1);
    T_ba = SE3(R1, t1);

    Tnaught = SE3();
    T_0d = SE3(T_0e.T*T_ed.T);
    T_0c = SE3(T_0d.T*T_dc.T);
    T_0b = SE3(T_0c.T*T_cb.T);
    T_0a = SE3(T_0b.T*T_ba.T);

    alpha_joint_frames = [Tnaught, T_0e, T_0d, T_0c, T_0b, T_0a];
    
    %% ---------- TWISTS ----------
    [TW, T0] = alphaArm.twists();
    Slist = [];
    for i = 1:length(TW)
        tw = TW(i).S;
        Slist = [Slist tw];
    end
    
    %% ---------- LINK FRAMES ----------
    rpy0 = [0 0 0];
    R0 = rpy2r(rpy0);
    T_link0_from_jointE = SE3(R0, BaseL.r);
    T_0_L0 = SE3(T_0e.T*T_link0_from_jointE.T);
        
    T_link1_from_jointE = SE3(R0, Link1.r);
    T_0_L1 = SE3(T_0e.T*T_link1_from_jointE.T);
    
    T_link2_from_jointD = SE3(R0, Link2.r);
    T_0_L2 = SE3(T_0d.T*T_link2_from_jointD.T);
    
    T_link3_from_jointC = SE3(R0, Link3.r);
    T_0_L3 = SE3(T_0c.T*T_link3_from_jointC.T);
    
    T_link4_from_jointB = SE3(R0, Link4.r);
    T_0_L4 = SE3(T_0b.T*T_link4_from_jointB.T);
    
    T_ee_from_jointA = SE3(R0, [0 -45 0]); % jaw1 x=-10, jaw2 x=-10
    T_0_ee = SE3(T_0_L4.T*T_ee_from_jointA.T);
    
    alpha_link_frames = [T_0_L0, T_0_L1, T_0_L2, T_0_L3, T_0_L4, T_0_ee];
    
    %% ---------- Relative link frames (M) ----------
    num_links = length(alpha_link_frames);
    inv_a_link_frames = [];
    M_backward = [];
    M_forward = [];
    
    %T_L1_0 = T_0_L1.inv;
    for i = 1:num_links
        inv_a_link_frames = [inv_a_link_frames alpha_link_frames(i).inv];
    end
    
    for j = 2:num_links
        M_j_jminus1 = SE3(inv_a_link_frames(j).T*alpha_link_frames(j-1).T);
        M_backward = [M_backward M_j_jminus1];
        
        M_jminus1_j = SE3(inv_a_link_frames(j-1).T*alpha_link_frames(j).T);
        M_forward = [M_forward M_jminus1_j];
    end
    
    Mlist = cat(3, SE3(T_0_L0.inv).T, M_forward(1).T, M_forward(2).T, M_forward(3).T, M_forward(4).T, M_forward(5).T);
    
    %% ---------- Spatial Inertia Matrices Gi ----------
    Gi_matrices = [];
    
    for i = 1:5
        link = alphaArm.links(i);
        G_i = [link.I(1,:) 0 0 0; link.I(2,:) 0 0 0; link.I(3,:) 0 0 0; ...
            0 0 0 link.m 0 0; 0 0 0 0 link.m 0; 0 0 0 0 0 link.m;];
        Gi_matrices = [Gi_matrices G_i];
    end
    
    Glist = cat(3, Gi_matrices(1), Gi_matrices(2), Gi_matrices(3), Gi_matrices(4), Gi_matrices(5));

    %% ---------- VIEWING ----------
%     hold on
% %     trplot(Tnaught.T, 'color', 'k', 'frame', 'base')
% %     trplot(T_0e.T, 'color', 'k', 'frame', 'Joint E')
% %     trplot(T_0d.T, 'color', 'k', 'frame', 'Joint D')
% %     trplot(T_0c.T, 'color', 'k', 'frame', 'Joint C')
% %     trplot(T_0b.T, 'color', 'k', 'frame', 'Joint B')
% %     trplot(T_0a.T, 'color', 'k', 'frame', 'Joint A')
% %     trplot(T_link0_from_joint0.T, 'color', 'm', 'frame', 'Link 0')
%     xlabel('X')
%     ylabel('Y')
%     grid on
end

