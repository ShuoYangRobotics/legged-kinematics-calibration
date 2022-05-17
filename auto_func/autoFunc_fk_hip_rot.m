function fk_hip_rot = autoFunc_fk_hip_rot(in1,lc,in3)
%autoFunc_fk_hip_rot
%    FK_HIP_ROT = autoFunc_fk_hip_rot(IN1,LC,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    16-May-2022 20:35:42

t1 = in1(1,:);
t3 = cos(t1);
t4 = sin(t1);
fk_hip_rot = reshape([1.0,0.0,0.0,0.0,t3,t4,0.0,-t4,t3],[3,3]);
