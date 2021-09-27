syms t fl1 fl2 d l th1(t) th2(t) th3(t) q1 q2 q3 dq1 dq2 dq3


pj0 = [fl1;-fl2;0];
pj1 = [fl1;-fl2-d*cos(th1);-d*sin(th1)];
pj2 = [fl1-l*sin(th2);
       -fl2-d*cos(th1)+l*cos(th2)*sin(th1);
       -d*sin(th1)-l*cos(th2)*cos(th1)];
pj3 = [fl1-l*sin(th2)-l*sin(th2+th3);
       -fl2-d*cos(th1)+(l*cos(th2)+l*cos(th2+th3))*sin(th1);
       -d*sin(th1)-(l*cos(th2)+l*cos(th2+th3))*cos(th1)];
   
%simplify(jacobian(pj3,[th1;th2;th3]))
J = [0,  -l*(cos(th2 + th3) + cos(th2)),          -l*cos(th2 + th3);
d*sin(th1) + cos(th1)*(l*cos(th2 + th3) + l*cos(th2)), -l*sin(th1)*(sin(th2 + th3) + sin(th2)), -l*sin(th2 + th3)*sin(th1);
sin(th1)*(l*cos(th2 + th3) + l*cos(th2)) - d*cos(th1),  l*cos(th1)*(sin(th2 + th3) + sin(th2)),  l*sin(th2 + th3)*cos(th1)];

% J = simplify(jacobian(pj3,[th1;th2;th3]));
dJ = diff(J,t);

out_p = subs(pj3,[th1(t), th2(t), th3(t)],[q1,q2,q3]);
out_J = subs(J,[th1(t), th2(t), th3(t)],[q1,q2,q3]);
out_dJ = subs(dJ,[th1(t), th2(t), th3(t),diff(th1(t), t),diff(th2(t), t),diff(th3(t), t)],[q1,q2,q3,dq1,dq2,dq3]);

param_init;
derive_dynamics;
J = [                                               0,         -l*(cos(q2 + q3) + cos(q2)),         -l*cos(q2 + q3);
cos(q1)*(l*cos(q2 + q3) + l*cos(q2)) + d*sin(q1), -l*sin(q1)*(sin(q2 + q3) + sin(q2)), -l*sin(q2 + q3)*sin(q1);
sin(q1)*(l*cos(q2 + q3) + l*cos(q2)) - d*cos(q1),  l*cos(q1)*(sin(q2 + q3) + sin(q2)),  l*sin(q2 + q3)*cos(q1)];

dJ = [                                                                                                              0,                                                    l*(sin(q2 + q3)*(dq2 + dq3) + dq2*sin(q2)),                                        l*sin(q2 + q3)*(dq2 + dq3);
d*dq1*cos(q1) - dq1*sin(q1)*(l*cos(q2 + q3) + l*cos(q2)) - cos(q1)*(dq2*l*sin(q2) + l*sin(q2 + q3)*(dq2 + dq3)), - l*sin(q1)*(cos(q2 + q3)*(dq2 + dq3) + dq2*cos(q2)) - dq1*l*cos(q1)*(sin(q2 + q3) + sin(q2)), - l*cos(q2 + q3)*sin(q1)*(dq2 + dq3) - dq1*l*sin(q2 + q3)*cos(q1);
dq1*cos(q1)*(l*cos(q2 + q3) + l*cos(q2)) - sin(q1)*(dq2*l*sin(q2) + l*sin(q2 + q3)*(dq2 + dq3)) + d*dq1*sin(q1),   l*cos(q1)*(cos(q2 + q3)*(dq2 + dq3) + dq2*cos(q2)) - dq1*l*sin(q1)*(sin(q2 + q3) + sin(q2)),   l*cos(q2 + q3)*cos(q1)*(dq2 + dq3) - dq1*l*sin(q2 + q3)*sin(q1)];

p =       [                      fl1 - l*sin(q2 + q3) - l*sin(q2);
sin(q1)*(l*cos(q2 + q3) + l*cos(q2)) - d*cos(q1) - fl2;
    - cos(q1)*(l*cos(q2 + q3) + l*cos(q2)) - d*sin(q1)];
matlabFunction(p,...
    'file','fk.m',...
    'vars',{q1,q2,q3},...
    'outputs',{'out_p'});
matlabFunction(J,...
    'file','Jb.m',...
    'vars',{q1,q2,q3},...
    'outputs',{'out_J'});
matlabFunction(dJ,...
    'file','dJb.m',...
    'vars',{q1,q2,q3, dq1, dq2, dq3},...
    'outputs',{'out_dJ'});