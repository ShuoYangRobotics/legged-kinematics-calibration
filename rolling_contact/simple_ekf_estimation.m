function [est_state_list,est_state_time] = simple_ekf_estimation(traj_t, state_init, ...
    gyro_IMU, pos_mocap, orient_mocap, vel_mocap, joint_ang, joint_vel, foot_force, param)

traj_len = floor(max(size(traj_t)));
% traj_len = 7000;
est_state_size = param.rho_opt_size*2*param.num_leg;
est_state_list = zeros(est_state_size, traj_len);
est_state_time = zeros(1,traj_len);
% estimation covariance
P = 0.1*eye(est_state_size);
% process noise covariance
Q = diag([1*ones(param.rho_opt_size*param.num_leg,1);
          1*ones(param.rho_opt_size*param.num_leg,1)]);
% measurement noise covariance, only consider x velocity
R = diag(0.001*ones(param.num_leg*2,1));
% save initial state
est_state_list(:,1) = state_init;

% step by step estimation
for i=2:traj_len
    t = traj_t(i);
    prev_t = traj_t(i-1);
    dt = t - prev_t;
    est_state_time(:,i) = t;
    % process update    
    % integrate state
    est_state_list(:,i) = simple_ekf_process(est_state_list(:,i-1), foot_force.Data(i,:), dt, param);
    [F,G] = simple_ekf_process_jac(dt, param);
    
    
    tmp_idxs = find(vel_mocap.Time-t>=0);
    % interpolate to get body velocity
    mocap_t1 = vel_mocap.Time(tmp_idxs(1)-1);
    mocap_t2 = vel_mocap.Time(tmp_idxs(1));
    alpha = (t-mocap_t1)/(mocap_t2-mocap_t1);
    qi = quatinterp(orient_mocap.Data(tmp_idxs(1)-1,:),orient_mocap.Data(tmp_idxs(1),:), alpha,'slerp');
    R_er = quat2rotm(qi);
    pi = (1-alpha)*pos_mocap.Data(tmp_idxs(1)-1,:) + alpha*pos_mocap.Data(tmp_idxs(1),:);    
    vi = (1-alpha)*vel_mocap.Data(tmp_idxs(1)-1,:) + alpha*vel_mocap.Data(tmp_idxs(1),:);
    
    % interpolate to get IMU angular velocity
    tmp_idxs = find(gyro_IMU.Time-t>=0);
    % interpolate to get state(mocap pos and orientation)
    imu_t1 = gyro_IMU.Time(tmp_idxs(1)-1);
    imu_t2 = gyro_IMU.Time(tmp_idxs(1));
    alpha = (t-imu_t1)/(imu_t2-imu_t1);
    wi = (1-alpha)*gyro_IMU.Data(tmp_idxs(1)-1,:) + alpha*gyro_IMU.Data(tmp_idxs(1),:);

    
    % measure x y velocity
    z = [vi(1:2)';vi(1:2)';vi(1:2)';vi(1:2)'];
    % the predict output
    zhat = simple_ekf_measurement(est_state_list(:,i), R_er, ...
        wi, joint_ang.Data(i,:), joint_vel.Data(i,:), param);

    H = simple_ekf_measurement_jac(est_state_list(:,i), R_er, ...
        wi, joint_ang.Data(i,:), joint_vel.Data(i,:), param);
    
    innov = z-zhat;
    % R is very large for non contact foot
    contacts = zeros(param.num_leg,1);
    diff_forces = zeros(param.num_leg,1);
    var_forces = zeros(param.num_leg,1);
    if (i > 5)
        var_forces = var(foot_force.Data(i-5:i,:));
    end
    % construct R
    for j = 1:param.num_leg
        no_contact = 1-1/(1+exp(-1*(foot_force.Data(i,j)-80)));
        contacts(j) = 1- no_contact;
        diff_force = foot_force.Data(i,j) - foot_force.Data(i-1,j);
        diff_forces(j) = diff_force;
        R((j-1)*2+1:(j-1)*2+2,(j-1)*2+1:(j-1)*2+2) = ...
            eye(2)*(5000*no_contact + 5000*abs(var_forces(j))^2 + 0.0001);
        Q((j-1)*param.rho_opt_size+1+param.rho_opt_size*param.num_leg:(j-1)*param.rho_opt_size+param.rho_opt_size+param.rho_opt_size*param.num_leg,...
          (j-1)*param.rho_opt_size+1+param.rho_opt_size*param.num_leg:(j-1)*param.rho_opt_size+param.rho_opt_size+param.rho_opt_size*param.num_leg) = 5*contacts(j)+0.0001;
    end
    
    P = F*P*F' + Q;
    Sinv = inv(H*P*H'+R);
    mask = [];
    for j = 1:param.num_leg
        overfit_test = innov((j-1)*2+1:(j-1)*2+2)'*Sinv((j-1)*2+1:(j-1)*2+2,(j-1)*2+1:(j-1)*2+2)...
                        *innov((j-1)*2+1:(j-1)*2+2);
        if overfit_test < 5e-7
            mask = [mask (j-1)*2+1 (j-1)*2+2];
        end

    end
    if i == 4000
        p = 0;
    end
    
    pH = H(mask,:);
    pR = R(mask,mask);
   
    K = P*pH'*inv(pH*P*pH'+pR);
    delta_x = K*(innov(mask));
    if (norm(innov) > 1e5)
        display('something wrong');
    end
    P = (eye(est_state_size) - K*pH)*P;
    if (size(delta_x,2) > 0)
        est_state_list(:,i) = est_state_list(:,i) + delta_x;
    end
%     delta_x
%     est_state_list(:,i)
end

end