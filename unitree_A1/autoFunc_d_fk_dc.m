function d_fk_dc = autoFunc_d_fk_dc(in1,in2,in3)
%AUTOFUNC_D_FK_DC
%    D_FK_DC = AUTOFUNC_D_FK_DC(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    01-Sep-2021 23:06:51

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = cos(q1);
t3 = sin(q1);
t4 = q2+q3;
t5 = cos(t4);
t6 = sin(t4);
d_fk_dc = reshape([t5,t3.*t6,-t2.*t6,0.0,t2,t3,t6,-t3.*t5,t2.*t5],[3,3]);
