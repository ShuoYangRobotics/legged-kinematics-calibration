function tau = fcn(q,dq,d, l, fl1, fl2, Slist, Mlist, Glist,t, ref_p_list, ref_t_list)

q1 = q(1);
q2 = q(2);
q3 = q(3);

dq1 = dq(1);
dq2 = dq(2);
dq3 = dq(3);

J = [                                               0,         -l*(cos(q2 + q3) + cos(q2)),         -l*cos(q2 + q3);
cos(q1)*(l*cos(q2 + q3) + l*cos(q2)) + d*sin(q1), -l*sin(q1)*(sin(q2 + q3) + sin(q2)), -l*sin(q2 + q3)*sin(q1);
sin(q1)*(l*cos(q2 + q3) + l*cos(q2)) - d*cos(q1),  l*cos(q1)*(sin(q2 + q3) + sin(q2)),  l*sin(q2 + q3)*cos(q1)];

dJ = [                                                                                                              0,                                                    l*(sin(q2 + q3)*(dq2 + dq3) + dq2*sin(q2)),                                        l*sin(q2 + q3)*(dq2 + dq3);
d*dq1*cos(q1) - dq1*sin(q1)*(l*cos(q2 + q3) + l*cos(q2)) - cos(q1)*(dq2*l*sin(q2) + l*sin(q2 + q3)*(dq2 + dq3)), - l*sin(q1)*(cos(q2 + q3)*(dq2 + dq3) + dq2*cos(q2)) - dq1*l*cos(q1)*(sin(q2 + q3) + sin(q2)), - l*cos(q2 + q3)*sin(q1)*(dq2 + dq3) - dq1*l*sin(q2 + q3)*cos(q1);
dq1*cos(q1)*(l*cos(q2 + q3) + l*cos(q2)) - sin(q1)*(dq2*l*sin(q2) + l*sin(q2 + q3)*(dq2 + dq3)) + d*dq1*sin(q1),   l*cos(q1)*(cos(q2 + q3)*(dq2 + dq3) + dq2*cos(q2)) - dq1*l*sin(q1)*(sin(q2 + q3) + sin(q2)),   l*cos(q2 + q3)*cos(q1)*(dq2 + dq3) - dq1*l*sin(q2 + q3)*sin(q1)];

p =       [                      fl1 - l*sin(q2 + q3) - l*sin(q2);
sin(q1)*(l*cos(q2 + q3) + l*cos(q2)) - d*cos(q1) - fl2;
    - cos(q1)*(l*cos(q2 + q3) + l*cos(q2)) - d*sin(q1)];

v = J*dq;

% dynamics
M = MassMatrix(q, Mlist, Glist, Slist);
c = VelQuadraticForces(q, dq, Mlist, Glist, Slist);
g = GravityForces(q, [0; 0; -9.8], Mlist, Glist, Slist);

% reference trajectory
[ref_p, ref_v, ref_a, ~] = quinticpolytraj(ref_p_list, ref_t_list, t);

Kp = diag([200;200;200]);
Kd = diag([100;100;100]);

tau = J'*(Kp*(ref_p-p)+Kd*(ref_v-v)) + J'*inv(J)'*M*inv(J)*(ref_a-dJ*dq) + c + g;


% tau = [0;0;0];
