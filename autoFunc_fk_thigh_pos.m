function fk_thigh_pos = autoFunc_fk_thigh_pos(in1,lc,in3)
%AUTOFUNC_FK_THIGH_POS
%    FK_THIGH_POS = AUTOFUNC_FK_THIGH_POS(IN1,LC,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    24-Feb-2022 11:42:22

d = in3(3,:);
lt = in3(4,:);
ox = in3(1,:);
oy = in3(2,:);
t1 = in1(1,:);
t2 = in1(2,:);
t4 = cos(t1);
t5 = cos(t2);
t6 = sin(t1);
fk_thigh_pos = [ox-(lt.*sin(t2))./2.0;oy+d.*t4+(lt.*t5.*t6)./2.0;d.*t6-(lt.*t4.*t5)./2.0];
