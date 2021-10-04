function jac = ekf_process_jac(state, meas, dt, param)
% the F function of the process model 

omega = meas(4:6);
acc = meas(1:3);
q_er = quaternion(state(4:7)');
R_er = quat2rotm(q_er);
jac = eye(param.state_size-1);

jac(1:3, 7:9) = eye(3)*dt;

jac(4:6,4:6) = eye(3) - skew(omega*dt);

jac(7:9,4:6) = -R_er*skew(acc*dt);


end