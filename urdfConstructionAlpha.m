% <joint name="${name}_joint5" type="revolute">
%           <parent link="${name}_base_link" />
%           <child link="${name}_shoulder_link" />
%           <origin xyz="0 0 0" rpy="0 0 3.141592" />
%           <axis xyz="0 0 1" />
%           <limit effort="9.0" lower="0.0" upper="6.2" velocity="0.5" />

clf; 

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
T_0a = SE3(T_0b.T*T_ba.T)

Joint_Frames = [Tnaught, T_0e, T_0d, T_0c, T_0b, T_0a];

hold on
trplot(Tnaught.T)
trplot(T_0e.T, 'color', 'c', 'frame', 'E', 'rviz')
trplot(T_0d.T, 'color', 'r', 'frame', 'D', 'rviz')
trplot(T_0c.T, 'color', 'g', 'frame', 'C', 'rviz')
trplot(T_0b.T, 'color', 'b', 'frame', 'B', 'rviz')
trplot(T_0a.T, 'color', 'm', 'frame', 'A', 'rviz')
grid on
% hold on

