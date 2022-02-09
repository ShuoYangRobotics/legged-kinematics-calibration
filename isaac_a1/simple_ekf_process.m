function next_state = simple_ekf_process(state, foot_force, dt, param)
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

[F,G] = simple_ekf_process_jac(dt, param);
            
u = zeros(4,1);
u(1) = foot_force(1);
u(2) = foot_force(2);
u(3) = foot_force(3);
u(4) = foot_force(4);

next_state = F*state + G*u;



end