%% Joint G 
% <joint name="bravo_axis_g" type="revolute">
%         <parent link="bravo_rs2_266_link"/>
%         <child link="bravo_rs2_217_link"/>
%         <origin rpy="0 0 0" xyz="0.0665 0 0.078"/>
%         <axis xyz="0 0 -1"/>
%         <limit effort="9.0" lower="0.0" upper="6.3" velocity="0.5"/>
 
clf; 

% Joint G frame wrt Base
t_g = [66.5 0 78];
rpy_g = [0 0 0];
R_g = rpy2r(rpy_g);
T_0g = SE3(R_g, t_g);

%% Joint F
%         <joint name="bravo_axis_f" type="revolute">
%         <parent link="bravo_rs2_217_link"/>
%         <child link="bravo_rs2_250_212_214_link"/>
%         <origin rpy="0 0 0" xyz="-0.046 0 0.0674"/>
%         <axis xyz="0 1 0"/>
%         <limit effort="9.0" lower="0.0" upper="3.5" velocity="0.5"/> 

t_f = [-46 0 67.4];
rpy_f = [0 0 0];
R_f = rpy2r(rpy_f);
T_gf = SE3(R_f, t_f);

%% Joint E
%  <joint name="bravo_axis_e" type="revolute">
%         <parent link="bravo_rs2_250_212_214_link"/>
%         <child link="bravo_rs2_268_link"/>
%         <origin rpy="0 0 0" xyz="-0.0052 0 -0.29055"/>
%         <axis xyz="0 1 0"/>
%         <limit effort="9.0" lower="0.0" upper="3.5" velocity="0.5"/>
        
t_e = [-5.2 0 -290.55];
rpy_e = [0 0 0];
R_e = rpy2r(rpy_e);
T_fe = SE3(R_e, t_e);

%% Joint D
% <joint name="bravo_axis_d" type="revolute">
%         <parent link="bravo_rs2_268_link"/>
%         <child link="bravo_rs2_214_link"/>
%         <origin rpy="0 0 3.14159" xyz="0.0408 0 0.09695"/>
%         <axis xyz="0 0 -1"/>
%         <limit effort="9.0" lower="0.0" upper="6.3" velocity="0.5"/>
%     </joint>

t_d = [40.8 0 96.95];
rpy_d = [0 0 pi]; % was -1.57075 -0.5 0
R_d = rpy2r(rpy_d);
T_ed = SE3(R_d, t_d);

%% Joint C
%      <joint name="bravo_axis_c" type="revolute">
%         <parent link="bravo_rs2_214_link"/>
%         <child link="bravo_rs2_161_link"/>
%         <origin rpy="0 0 0" xyz="-0.0408 0 0.063"/>
%         <axis xyz="0 1 0"/>
%         <limit effort="9.0" lower="0.0" upper="3.5" velocity="0.5"/>
%     </joint>
    
t_c = [-40.8 0 63]; 
rpy_c = [0 0 0];  
R_c = rpy2r(rpy_c);
T_dc = SE3(R_c, t_c);

%% Joint B
%  <joint name="bravo_axis_b" type="revolute">
%         <parent link="bravo_rs2_161_link"/>
%         <child link="bravo_rs2_180_link"/>
%         <origin rpy="0 3.14159 0" xyz="-0.0408 0 -0.08863"/>
%         <axis xyz="0 0 -1"/>
%         <limit effort="9.0" lower="0.0" upper="6.3" velocity="0.5"/>
%     </joint>

t_b = [-40.8 0 -88.63]; 
rpy_b = [0 pi 0];
R_b = rpy2r(rpy_b);
T_cb = SE3(R_b, t_b);

%% Jaws Joint
%         <joint name="bravo_jaws_joint" type="fixed">
%     <origin rpy="0 0 0" xyz=" 0 0 0.124"/>
%     <parent link="bravo_rs2_180_link"/>
%     <child link="bravo_jaws_base_link"/>
%   </joint>

t_jaws = [0 0 124];
rpy_jaws = [0 0 0];
R_jaws = rpy2r(rpy_jaws);
T_bjaws = SE3(R_jaws,t_jaws);

%% joint A
%     <joint name="bravo_axis_a" type="prismatic">
%         <origin rpy="0 0 0" xyz="0 0 0.009"/>
%         <parent link="bravo_jaws_base_link"/>
%         <child link="bravo_push_rod"/>
%         <axis xyz="0 0 1"/>
%         <limit effort="10" lower="0" upper="0.035" velocity="10"/>
%     </joint>

t_a = [0 0 9];
rpy_a = [0 0 0];
R_a = rpy2r(rpy_a);
T_jawsA = SE3(R_a, t_a);

%% Calculations
Tnaught = SE3();
p1 = T_0g.tv;
T_0f = SE3(T_0g.T*T_gf.T);
p2 = T_0f.tv;
T_0e = SE3(T_0f.T*T_fe.T);
p3 = T_0e.tv;
T_0d = SE3(T_0e.T*T_ed.T);
p4 = T_0d.tv;
T_0c = SE3(T_0d.T*T_dc.T);
p5 = T_0c.tv;
T_0b = SE3(T_0c.T*T_cb.T);
p6 = T_0b.tv;
T_0jaws = SE3(T_0b.T*T_bjaws.T);
p7 = T_0jaws.tv;
T_0a = SE3(T_0jaws.T*T_jawsA.T);
p8 = T_0a.tv;

hold on
trplot(Tnaught.T, 'frame', 'Base', 'rviz', 'length', 25)
trplot(T_0g.T, 'frame', 'G', 'rviz', 'length', 25)
trplot(T_0f.T, 'frame', 'F', 'rviz', 'length', 25)
trplot(T_0e.T, 'frame', 'E', 'rviz', 'length', 25)
trplot(T_0d.T, 'frame', 'D', 'rviz', 'length', 25)
trplot(T_0c.T, 'frame', 'C', 'rviz', 'length', 25)
trplot(T_0b.T, 'frame', 'B', 'rviz', 'length', 25)
trplot(T_0a.T, 'frame', 'A', 'rviz', 'length', 25)

plot3([0, p1(1), p2(1), p3(1), p4(1), p5(1), p6(1), p7(1), p8(1)],...
    [0, p1(2), p2(2), p3(2), p4(2), p5(2), p6(2), p7(2), p8(2)], ...
    [0, p1(3), p2(3), p3(3), p4(3), p5(3), p6(3), p7(3), p8(3)], '-') 

h = gcf;
grid on
daspect([1 1 1])
pbaspect([1 1 1])

% axis 
% hold on

