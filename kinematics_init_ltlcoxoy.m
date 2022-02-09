% this script derives forward kinematics for a Unitree A1 leg
% it first uses symbolic computation, then save the symbolic computation as
% matlab functions
% the matlab functions then outputs to C++ functions

% first add modern robotics tool box
addpath('unitree_A1/mr')

%% modify this section is parameters to be estimated are different
% % estimate cx cy cz
% parameters (align with paper)
%     -    ox, the x direction offset from body center to leg base
%     -    oy, the y direction offset from body center to leg base
%     -     d, the offset from thigh motor to calf motor
%     -    lt, the upper leg (thigh) length
%     -    lc, the lower leg (calf) length   
%     -    t1 t2 t3, the joint angles theta
%     -    dt1 dt2 dt3, the joint angular velocity
%     -    cx cy cz                                      (to be estimated)

syms ox oy d lt lc t1 t2 t3 dt1 dt2 dt3 cx cy cz real
% determine estimation size and fix size
theta = [t1;t2;t3];
rho_opt = [lt;lc;ox;oy];
rho_fix = [d];

% joint 1 w and q
w1 = [1;0;0];
pq1 = [ox;
       oy;
        0];
% joint 2 w and q
w2 = [0;1;0];
pq2 = [  ox;
         oy;
          0];
% joint 3 w and q      
w3 = [0;1;0];
pq3 = [  ox;
       oy+d;
        -lt];

% hip motor home position
p_hip = [   ox;
        oy+d/2;
             0];

% thigh link home position
p_thigh = [   ox;
            oy+d;
           -lt/2];

% calf link home position
p_calf = [   ox;
           oy+d;
          -lt-lc/2];

% end effector home position  
pf = [   ox;
       oy+d;
     -lt-lc];

% forward kinematics of hip motor
Slist = [[w1;-cross(w1,pq1)]];
M1 = [eye(3) p_hip; [0 0 0 1]];
fk_hip_sym = FKinSpace_sym(M1,Slist, [t1]);
fk_hip_pos = simplify(fk_hip_sym(1:3,4));
fk_hip_rot = simplify(fk_hip_sym(1:3,1:3));

% forward kinematics of thigh link
Slist = [[w1;-cross(w1,pq1)] [w2;-cross(w2,pq2)]];
M2 = [eye(3) p_thigh; [0 0 0 1]];
fk_thigh_sym = FKinSpace_sym(M2,Slist, [t1;t2]);
fk_thigh_pos = simplify(fk_thigh_sym(1:3,4));
fk_thigh_rot = simplify(fk_thigh_sym(1:3,1:3));

% forward kinematics of calf link
Slist = [[w1;-cross(w1,pq1)] [w2;-cross(w2,pq2)] [w3;-cross(w3,pq3)]];
M3 = [eye(3) p_calf; [0 0 0 1]];
fk_calf_sym = FKinSpace_sym(M3,Slist, theta);
fk_calf_pos = simplify(fk_calf_sym(1:3,4));
fk_calf_rot = simplify(fk_calf_sym(1:3,1:3));

% forward kinematics of end effector
Slist = [[w1;-cross(w1,pq1)] [w2;-cross(w2,pq2)] [w3;-cross(w3,pq3)]];
M4 = [eye(3) pf; [0 0 0 1]];
fk_pf_sym = FKinSpace_sym(M4,Slist, theta);

fk_pf_pos = simplify(fk_pf_sym(1:3,4));

% derivative of forward kinematics position wrt joint angles
d_fk_dt = simplify(jacobian(fk_pf_pos,theta))

% derivative of forward kinematics position wrt rho_opt
d_fk_drho = simplify(jacobian(fk_pf_pos,rho_opt))

% J = d_fk_dq is the Jacobian, now we need to further take its Jacobian wrt
% theta
% vectorize
dJ_dt = simplify(jacobian(d_fk_dt(:),theta))
% d_fk_dq is the Jacobian, now we need to further take its Jacobian wrt
% rho_opt
% vectorize
dJ_drho = simplify(jacobian(d_fk_dt(:),rho_opt))

%% according to the previous section, generate necessary matlab functions 

% save hip motor forward kinematics position and orientation
matlabFunction(fk_hip_pos,...
    'file','autoFunc_fk_hip_pos.m',...
    'vars',{theta, rho_opt,rho_fix},...
    'outputs',{'fk_hip_pos'});

matlabFunction(fk_hip_rot,...
    'file','autoFunc_fk_hip_rot.m',...
    'vars',{theta, rho_opt,rho_fix},...
    'outputs',{'fk_hip_rot'});

% save thigh link forward kinematics position and orientation
matlabFunction(fk_thigh_pos,...
    'file','autoFunc_fk_thigh_pos.m',...
    'vars',{theta, rho_opt,rho_fix},...
    'outputs',{'fk_thigh_pos'});

matlabFunction(fk_thigh_rot,...
    'file','autoFunc_fk_thigh_rot.m',...
    'vars',{theta, rho_opt,rho_fix},...
    'outputs',{'fk_thigh_rot'});

% save calf link forward kinematics position and orientation
matlabFunction(fk_calf_pos,...
    'file','autoFunc_fk_calf_pos.m',...
    'vars',{theta, rho_opt,rho_fix},...
    'outputs',{'fk_calf_pos'});

matlabFunction(fk_calf_rot,...
    'file','autoFunc_fk_calf_rot.m',...
    'vars',{theta, rho_opt,rho_fix},...
    'outputs',{'fk_calf_rot'});

% save end effector forward kinematics position
matlabFunction(fk_pf_pos,...
    'file','autoFunc_fk_pf_pos.m',...
    'vars',{theta, rho_opt,rho_fix},...
    'outputs',{'p_bf'});
% save end effector jacobian
matlabFunction(d_fk_dt,...
    'file','autoFunc_d_fk_dt.m',...
    'vars',{theta, rho_opt,rho_fix},...
    'outputs',{'jacobian'});
matlabFunction(d_fk_drho,...
    'file','autoFunc_d_fk_drho.m',...
    'vars',{theta, rho_opt,rho_fix},...
    'outputs',{'d_fk_drho'});
matlabFunction(dJ_dt,...
    'file','autoFunc_dJ_dt.m',...
    'vars',{theta, rho_opt,rho_fix},...
    'outputs',{'dJ_dq'});
matlabFunction(dJ_drho,...
    'file','autoFunc_dJ_drho.m',...
    'vars',{theta, rho_opt,rho_fix},...
    'outputs',{'dJ_drho'});

% a final test script shows that the generated fk functions work
% test_kinematics_function_generation_cxyz

%% put some necessary variables in param struct
param.rho_opt_size = size(rho_opt,1);
param.rho_fix_size = size(rho_fix,1);
param.rho_opt_str = '$l_t$; $l_c$; $o_x$; $o_y$';

param.num_leg = 4;
param.leg_name = ['FL','FR','RL', 'RR'];
param.all_leg = [1,2,3,4];
param.ox = [0.3,0.3,-0.3,-0.3];
param.oy = [0.15,-0.15,0.15,-0.15];
param.d = [0.08,-0.08,0.08,-0.08];
param.lt = 0.20;
param.lc = 0.22;

param.rho_opt_true = zeros(param.rho_opt_size,4);
param.rho_opt_true(:,1) = [param.lt;param.lc;param.ox(1);param.oy(1)];
param.rho_opt_true(:,2) = [param.lt;param.lc;param.ox(2);param.oy(2)];
param.rho_opt_true(:,3) = [param.lt;param.lc;param.ox(3);param.oy(3)];
param.rho_opt_true(:,4) = [param.lt;param.lc;param.ox(4);param.oy(4)];

param.rho_opt_init = zeros(param.rho_opt_size,4);
param.rho_opt_init(:,1) = [param.lt;param.lc;param.ox(1);param.oy(1)]+0.1*randn(param.rho_opt_size,1);
param.rho_opt_init(:,2) = [param.lt;param.lc;param.ox(2);param.oy(2)]+0.2*randn(param.rho_opt_size,1);
param.rho_opt_init(:,3) = [param.lt;param.lc;param.ox(3);param.oy(3)]+0.1*randn(param.rho_opt_size,1);
param.rho_opt_init(:,4) = [param.lt;param.lc;param.ox(4);param.oy(4)]+0.1*randn(param.rho_opt_size,1);

param.rho_fix = zeros(param.rho_fix_size,4);
param.rho_fix(:,1) = [param.d(1)];
param.rho_fix(:,2) = [param.d(2)];
param.rho_fix(:,3) = [param.d(3)];
param.rho_fix(:,4) = [param.d(4)];

