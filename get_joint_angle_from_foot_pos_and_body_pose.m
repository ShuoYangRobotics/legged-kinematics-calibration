function angle = get_joint_angle_from_foot_pos_and_body_pose(body_p, body_q, foot_pos, id, init_angle, param)
% body_q must be quaternion
R_er = quat2rotm(body_q);
next_prf = R_er'*(foot_pos - body_p);
angle = ik(next_prf, init_angle, [param.lc],[param.ox(id);param.oy(id);param.d(id);param.lt]);

end