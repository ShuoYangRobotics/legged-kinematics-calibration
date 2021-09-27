addpath(genpath(fullfile(pwd,'mr')));

%% calculate dynamic model
g_term = -9.8;
g = [0; 0; -9.8];
Ftip = zeros(6, 1); 

% data from https://github.com/unitreerobotics/unitree_ros/blob/master/robots/a1_description/xacro/const.xacro
hip_mass=0.696;
hip_com_x=-0.003311;
hip_com_y=0.000635;
hip_com_z=0.000031;
hip_ixx=   0.000469246;
hip_ixy=  -0.000009409;
hip_ixz=  -0.000000342;
hip_iyy=   0.000807490;
hip_iyz=  -0.000000466;
hip_izz=   0.000552929;

thigh_mass=1.013;
thigh_com_x=-0.003237;
thigh_com_y=-0.022327;
thigh_com_z=-0.027326;
thigh_ixx=0.005529065;
thigh_ixy=0.000004825;
thigh_ixz=0.000343869;
thigh_iyy=0.005139339;
thigh_iyz=0.000022448;
thigh_izz=0.001367788;

calf_mass=0.166;
calf_com_x=0.006435;
calf_com_y=0.0;
calf_com_z=-0.107388;
calf_ixx=  0.002997972;
calf_ixy=  0.0;
calf_ixz= -0.000141163;
calf_iyy=  0.003014022;
calf_iyz=  0.0;
calf_izz=  0.000032426;
% robot structure
% be careful about the position of these frames 
w1 = [1;0;0];
pq1 = [fl1;
      -fl2;
         0];
w2 = [0;1;0];
pq2 = [fl1;
      -fl2-d;
         0];
w3 = [0;1;0];
pq3 = [fl1;
      -fl2-d;
         -l];
pf = [fl1;
      -fl2-d;
         -l*2];
Slist = [[w1;-cross(w1,pq1)] [w2;-cross(w2,pq2)] [w3;-cross(w3,pq3)]];
% for FR
mirror = -1;
front_hind = 1;
M1 = [eye(3) pq1+[hip_com_x*front_hind;hip_com_y*mirror;hip_com_z]; [0 0 0 1]];
M2 = [eye(3) pq2+[thigh_com_x;thigh_com_y*mirror;thigh_com_z]; [0 0 0 1]];
M3 = [eye(3) pq3+[calf_com_x;calf_com_y;calf_com_z]; [0 0 0 1]];
M4 = [eye(3) pf; [0 0 0 1]];

M01 = M1;
M12 = inv(M1)*M2;
M23 = inv(M2)*M3;
M34 = inv(M3)*M4;

I1 = [hip_ixx        hip_ixy        hip_ixz;
      hip_ixy        hip_iyy        hip_iyz;
      hip_ixz        hip_iyz        hip_izz ];
        
G1 = [I1 zeros(3,3);
      zeros(3,3) diag([hip_mass, hip_mass, hip_mass])];
  
I2 = [thigh_ixx    thigh_ixy      thigh_ixx;
      thigh_ixy    thigh_iyy      thigh_iyz;
      thigh_ixz    thigh_iyz      thigh_izz ];
G2 = [I2 zeros(3,3);
      zeros(3,3) diag([thigh_mass, thigh_mass, thigh_mass])];
  
I3 = [calf_ixx    calf_ixy      calf_ixx;
      calf_ixy    calf_iyy      calf_iyz;
      calf_ixz    calf_iyz      calf_izz ];
G3 = [I3 zeros(3,3);
      zeros(3,3) diag([calf_mass, calf_mass, calf_mass])];

Glist = cat(3, G1, G2, G3);
Mlist = cat(3, M01, M12, M23, M34); 


