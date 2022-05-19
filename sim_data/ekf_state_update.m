function new_state=ekf_state_update(x, dx)

new_state = x;
new_state(1:3) = new_state(1:3) + dx(1:3);
new_q = quat_expmap(quaternion(new_state(4:7)'), dx(4:6));
[new_state(4),new_state(5),new_state(6),new_state(7)] = parts(new_q);
new_state(8:end) = new_state(8:end) + dx(7:end);

end