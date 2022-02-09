function [F,G] = simple_ekf_process_jac(dt, param)
% the process model of the simple ekf
% x_{n+1} = f(x_n, u)
% process 
% [drho1        [0     0   0   0   1  0   0   0   [   rho1        [ 0
%  drho2         0     0   0   0   0   1  0   0       rho2          0
%  drho3         0     0   0   0   0   0   1  0       rho3          0
%  drho4   =     0     0   0   0   0   0   0   1      rho4   +      0
%  ddrho1       -k     0   0   0   -c   0   0   0    drho1        -a*f1
%  ddrho2        0    -k   0   0   0   -c   0   0    drho2        -a*f2
%  ddrho3        0     0  -k   0   0   0   -c   0    drho3        -a*f3
%  ddrho4]       0     0   0  -k   0   0   0   -c]   drho4]       -a*f4]

% k = param.simple_ekf_process_position_ratio;
% c = param.simple_ekf_process_velocity_ratio;
% process_mtx = zeros(param.rho_opt_size*2*param.num_leg,param.rho_opt_size*2*param.num_leg);
% for i=1:param.num_leg
%     process_mtx(param.rho_opt_size*(i-1)+1:param.rho_opt_size*(i-1)+param.rho_opt_size, param.rho_opt_size*(i-1)+1+param.rho_opt_size*param.num_leg:param.rho_opt_size*(i-1)+param.rho_opt_size+param.rho_opt_size*param.num_leg) = eye(param.rho_opt_size);
%     process_mtx(param.rho_opt_size*(i-1)+1+param.rho_opt_size*param.num_leg:param.rho_opt_size*(i-1)+param.rho_opt_size+param.rho_opt_size*param.num_leg, param.rho_opt_size*(i-1)+1:param.rho_opt_size*(i-1)+param.rho_opt_size) = -k*eye(param.rho_opt_size);
%     process_mtx(param.rho_opt_size*(i-1)+1+param.rho_opt_size*param.num_leg:...
%         param.rho_opt_size*(i-1)+param.rho_opt_size+param.rho_opt_size*param.num_leg, ...
%         param.rho_opt_size*(i-1)+1+param.rho_opt_size*param.num_leg:...
%         param.rho_opt_size*(i-1)+param.rho_opt_size+param.rho_opt_size*param.num_leg) = -c*eye(param.rho_opt_size);
% end

process_mtx = zeros(param.rho_opt_size*2*param.num_leg);

control_mtx = zeros(param.rho_opt_size*2*param.num_leg,4);

a = param.simple_ekf_process_force_ratio;
for i=1:param.num_leg
    control_mtx(param.rho_opt_size*param.num_leg+i*param.rho_opt_size,1) = -a; 
end

F = eye(param.rho_opt_size*2*param.num_leg) + process_mtx*dt;


G = control_mtx*dt;
end