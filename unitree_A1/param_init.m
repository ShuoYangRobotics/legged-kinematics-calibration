joint_damp = 0;

fl1 = 0.1805;  % leg_offset_x
fl2 = 0.047;   % leg_offset_y

d = 0.0838;  % thigh_offset
l = 0.2;     % thigh_length and calf_length

init_q1 = 0;
init_q2 = 60/180*pi;
init_q3 = -100/180*pi;

init_p = fk(init_q1,init_q2,init_q3);

% create a reference circular trajectory 

period = 3;  %3 seconds

tgt_p = init_p + [0.15;0;0];
mid_p = (tgt_p+init_p)/2+[0;0;0.08];

ref_p_list = [init_p mid_p tgt_p init_p];
ref_t_list = [0 1 2 3];
[q, qd, qdd, pp] = cubicpolytraj(ref_p_list, ref_t_list, 3);



