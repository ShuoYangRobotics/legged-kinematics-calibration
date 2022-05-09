function dJ_dq = autoFunc_dJ_dt(in1,lc,in3)
%autoFunc_dJ_dt
%    dJ_dq = autoFunc_dJ_dt(IN1,LC,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    09-May-2022 15:50:09

d = in3(3,:);
lt = in3(4,:);
t1 = in1(1,:);
t2 = in1(2,:);
t3 = in1(3,:);
t5 = cos(t1);
t6 = cos(t2);
t7 = cos(t3);
t8 = sin(t1);
t9 = sin(t2);
t10 = sin(t3);
t11 = t2+t3;
t12 = lt.*t6;
t13 = cos(t11);
t14 = lt.*t9;
t15 = sin(t11);
t16 = lc.*t13;
t17 = lc.*t15;
t18 = t5.*t16;
t19 = t8.*t16;
t20 = t5.*t17;
t21 = t8.*t17;
t25 = t12+t16;
t26 = t14+t17;
t22 = -t19;
t23 = -t20;
t24 = -t21;
t27 = t5.*t26;
t28 = t8.*t26;
t29 = -t27;
t30 = -t28;
dJ_dq = reshape([0.0,-d.*t5-t8.*t12-lc.*t6.*t7.*t8+lc.*t8.*t9.*t10,-d.*t8+t5.*t12+lc.*t5.*t6.*t7-lc.*t5.*t9.*t10,0.0,t29,t30,0.0,t23,t24,0.0,t29,t30,t26,-t8.*t25,t5.*t25,t17,t22,t18,0.0,t23,t24,t17,t22,t18,t17,t22,t18],[9,3]);
