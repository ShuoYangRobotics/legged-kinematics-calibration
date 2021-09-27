syms t fx fy d l q1 q2 q3 dq1 dq2 dq3

w1 = [1;0;0];
pq1 = [fx;
       fy;
        0];
w2 = [0;1;0];
pq2 = [  fx;
       fy+d;
          0];
w3 = [0;1;0];
pq3 = [  fx;
       fy+d;
          -l];
pf = [   fx;
       fy+d;
       -l*2];
Slist = [[w1;-cross(w1,pq1)] [w2;-cross(w2,pq2)] [w3;-cross(w3,pq3)]];

M4 = [eye(3) pf; [0 0 0 1]];

fk_sym = FKinSpace_sym(M4,Slist, [q1;q2;q3]);
simplify(fk_sym(1:3,4))

fk_derive = fk_sym(1:3,4)
% fk_derive = [            fx - l*sin(q2 + q3) - l*sin(q2)
% fy + d*cos(q1) + l*cos(q2)*sin(q1) + l*cos(q2)*cos(q3)*sin(q1) - l*sin(q1)*sin(q2)*sin(q3)
%      d*sin(q1) - l*cos(q1)*cos(q2) - l*cos(q1)*cos(q2)*cos(q3) + l*cos(q1)*sin(q2)*sin(q3)]

simplify(jacobian(fk_derive,[q1;q2;q3]))

