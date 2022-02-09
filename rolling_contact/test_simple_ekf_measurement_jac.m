% test simple_ekf_measurement_jac

%
state = [    0.0003
   -0.0030
   -0.0000
   -0.0000
   -0.0002
   -0.0004
   -0.0002
   -0.0002];

R_er = [    0.9999    0.0110   -0.0018
   -0.0110    0.9999    0.0020
    0.0019   -0.0019    1.0000];

omega = [0.0100    0.0004   -0.0004];

joint_ang_list =[    0.1842
    1.0868
   -2.5976
   -0.2197
    1.1093
   -2.5922
    0.2293
    1.0795
   -2.5677
   -0.1343
    1.1324
   -2.5790];

joint_vel_list =[    0.0000
    0.0000
         0
    0.0000
    0.0000
   -0.0000
   -0.0013
   -0.0013
   -0.0014
    0.0000
   -0.0000
    0.0000];

zhat = simple_ekf_measurement(state, R_er, ...
    wi, joint_ang_list, joint_vel_list, param)

H = simple_ekf_measurement_jac(state, R_er, ...
    wi, joint_ang_list, joint_vel_list, param)

dstate = 0.01*rand(param.rho_opt_size*2*param.num_leg,1);
% dstate = [0.01*rand(param.rho_opt_size*param.num_leg,1);
%           zeros(param.rho_opt_size*param.num_leg,1);];
% dstate = [zeros(param.rho_opt_size*param.num_leg,1);
%           0.01*rand(param.rho_opt_size*param.num_leg,1)];

zhat2 = simple_ekf_measurement(state+dstate, R_er, ...
    wi, joint_ang_list, joint_vel_list, param);

zhat3 = zhat + H*dstate;

zhat2 - zhat3



% angle = joint_ang_list(1:3);
% av = 0.5*rand(3,1);
% rho = 0
% df_drho = autoFunc_d_fk_drho(angle,rho,param.rho_fix(:,i));
% v = -R_er*skew(omega)

% J_rf = autoFunc_d_fk_dt(angle,rho,param.rho_fix(:,1))*av
% drho = 0.01
% J_rf2 = autoFunc_d_fk_dt(angle,rho+drho,param.rho_fix(:,1))*av
% 
% H = autoFunc_dJ_drho(angle,rho,param.rho_fix(:,1))
% 
% J_rf3 = J_rf + kron(av',eye(3))*H*drho
% 
% J_rf2 - J_rf3
