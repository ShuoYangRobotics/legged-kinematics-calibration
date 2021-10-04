function next_state = ekf_process(state, meas, dt, param)
% the process model of the ekf
% x_{n+1} = f(x_n, u)

omega = meas(4:6);
acc = meas(1:3);
p_er = state(1:3);
q_er = quaternion(state(4:7)');
v_er = state(8:10);
R_er = quat2rotm(q_er);

next_state = state;

next_state(1:3) = p_er + v_er*dt + 0.5*(R_er*acc)*dt*dt;
new_q = quat_expmap(q_er, omega*dt);
[new_state(4),new_state(5),new_state(6),new_state(7)] = parts(new_q);
next_state(8:10) = v_er + (R_er*acc)*dt;

end