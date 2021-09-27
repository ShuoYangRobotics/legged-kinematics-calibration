% 08-08 explain: This script generates forward kinematics of unitree A1 leg
% parameters
%     -    fx, the x direction offset from body center to leg base
%     -    fy, the y direction offset from body center to leg base
%     -     d, the offset from thigh motor to calf motor
%     -    ul, the upper leg length
%     -    ll, the lower leg length
%     -    cx, cy, cz, the foot contact location offset, ideally are all 0
%     -    q1 q2 q3, the joint angles
%     -    dq1 dq2 dq3, the joint angular velocity


syms t fx fy d ul ll cx cy cz q1 q2 q3 dq1 dq2 dq3
addpath('mr')
% joint 1
w1 = [1;0;0];
pq1 = [fx;
       fy;
        0];
% joint 2
w2 = [0;1;0];
pq2 = [  fx;
         fy;
          0];
% joint 3      
w3 = [0;1;0];
pq3 = [  fx;
       fy+d;
        -ul];
% end effector position, consider contact offset    
pf = [   fx+cx;
       fy+d+cy;
     -ul-ll+cz];
% forward kinematics  
Slist = [[w1;-cross(w1,pq1)] [w2;-cross(w2,pq2)] [w3;-cross(w3,pq3)]];
M4 = [eye(3) pf; [0 0 0 1]];
fk_sym = FKinSpace_sym(M4,Slist, [q1;q2;q3]);

% forward kinematics position
fk_derive = simplify(fk_sym(1:3,4));

% derivative of forward kinematics position wrt joint angles
d_fk_dq = simplify(jacobian(fk_derive,[q1;q2;q3]))

% derivative of forward kinematics position wrt contact locations
d_fk_dc = simplify(jacobian(fk_derive,[cx;cy;cz]))

% show that in fk the contact location parameters are linear
simplify(fk_derive - d_fk_dc*[cx;cy;cz]) % the result does not contain cx cy cz 

% so the rank of d_fk_dc is important, let check for different values, what
% are the rank of d_fk_dc
% [X,Y,Z] = meshgrid(-pi:0.5:pi,-pi:0.5:pi,-pi:0.5:pi);
% X = X(:); Y = Y(:); Z = Z(:);
% 
% for i=1:size(X,1)
%     rank2 = rank(eval(subs(d_fk_dc,[q1;q2;q3],[X(i);Y(i);Z(i)])));
%     if rank2 == 3
%         plot3(X(i),Y(i),Z(i),'o', 'Color','r')
%     else
%         plot3(X(i),Y(i),Z(i),'o', 'Color','b')
%     end
%     hold on;
% end
% for all these values the rank is full 

%% d_fk_dq is the Jacobian, now we need to further take its Jacobian wrt q1
% q2 q3

% vectorize
dJ_dq = simplify(jacobian(d_fk_dq(:),[q1;q2;q3]))
dJ_dq_sub = subs(dJ_dq,[fx fy d ul ll cx cy cz],[0.3 0.15 0.08 0.2 0.2 0.0 0.0 0.0]);

% check the linearity (good linearity)
num_q = randn(3,1);
perturb_q = 0.01*randn(3,1);
%J(num_q+perturb_q)
% substitute all syms other than q1 q2 q3
J_fk = subs(d_fk_dq,[fx fy d ul ll cx cy cz],[0.3 0.15 0.08 0.2 0.2 0.0 0.0 0.0]);
Jqp = eval(subs(J_fk,[q1;q2;q3],num_q+perturb_q))
%J(num_q)
Jq = eval(subs(J_fk,[q1;q2;q3],num_q))
%dJdq(num_q)
dJdq = eval(subs(dJ_dq_sub,[q1;q2;q3],num_q))
% convert vectorized back to correct shape
Jqp - (Jq+reshape(dJdq*perturb_q,3,3))

%% d_fk_dq is the Jacobian, now we need to further take its Jacobian wrt 
% cx cy cz ( we call pho = [cx;cy;cz])
% substitute all syms other than q1 q2 q3
num_q1=randn; num_q2=randn; num_q3=randn;

% vectorize
dJ_dpho = simplify(jacobian(d_fk_dq(:),[cx;cy;cz]))
dJ_dpho_sub = subs(dJ_dpho,[fx fy d ul ll q1 q2 q3],[0.3 0.15 0.08 0.2 0.2 num_q1 num_q2 num_q3]);

% check the linearity (very good linearity)
num_pho = zeros(3,1);
perturb_pho = 0.1*randn(3,1);
%J(num_pho+perturb_pho)
J_fpho = subs(d_fk_dq,[fx fy d ul ll q1 q2 q3],[0.3 0.15 0.08 0.2 0.2 num_q1 num_q2 num_q3]);
Jphop = eval(subs(J_fpho,[cx;cy;cz],num_pho+perturb_pho))
%J(num_pho)
Jpho = eval(subs(J_fpho,[cx;cy;cz],num_pho))
%dJdc(num_c)
dJdpho = eval(subs(dJ_dpho_sub,[cx;cy;cz],num_pho))
% convert vectorized back to correct shape
Jphop - (Jpho+reshape(dJdpho*perturb_pho,3,3))


% save necessary functions as matlab functons
% we save 
%   - fk_derive
%   - d_fk_dq or (dJ)
%   - d_fk_dc
%   - dJ_dq
%   - dJ_dpho
q = [q1;q2;q3];
rho_opt = [cx;cy;cz];
rho_fix = [fx;fy;d;ul;ll];

matlabFunction(fk_derive,...
    'file','autoFunc_fk_derive.m',...
    'vars',{q, rho_opt,rho_fix},...
    'outputs',{'p_bf'});
matlabFunction(d_fk_dq,...
    'file','autoFunc_d_fk_dq.m',...
    'vars',{q, rho_opt,rho_fix},...
    'outputs',{'jacobian'});
matlabFunction(d_fk_dc,...
    'file','autoFunc_d_fk_dc.m',...
    'vars',{q, rho_opt,rho_fix},...
    'outputs',{'d_fk_dc'});
matlabFunction(dJ_dq,...
    'file','autoFunc_dJ_dq.m',...
    'vars',{q, rho_opt,rho_fix},...
    'outputs',{'dJ_dq'});
matlabFunction(dJ_dpho,...
    'file','autoFunc_dJ_dpho.m',...
    'vars',{q, rho_opt,rho_fix},...
    'outputs',{'dJ_dpho'});
% test 
autoFunc_test;
%finally, convert them to C code using coder