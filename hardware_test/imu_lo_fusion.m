function [pos_ts,vel_ts] = imu_lo_fusion(accel_IMU, lo_v_ts, contact_flags)

traj_len = length(lo_v_ts.Data);

% estimate pos and velocity only

state_init = [zeros(3,1);zeros(3,1)];
est_state_size = 6;
est_state_list = zeros(est_state_size, traj_len);
est_state_time = zeros(1,traj_len);
% estimation covariance
P = 0.1*eye(est_state_size);
% process noise covariance
Q = diag([0.01*ones(3,1);
          1*ones(3,1)]);
% measurement noise covariance
R = diag(0.001*ones(3,1));
% save initial state
est_state_list(:,1) = state_init;


% for each time step 
for t_idx = 2:size(lo_v_ts.Time,1)
    t = lo_v_ts.Time(t_idx);
    dt = lo_v_ts.Time(t_idx) - lo_v_ts.Time(t_idx-1);
    
    % interpolate to get IMU a
    tmp_idxs = find(accel_IMU.Time-t>=0);
    imu_t1 = accel_IMU.Time(tmp_idxs(1)-1);
    imu_t2 = accel_IMU.Time(tmp_idxs(1));
    alpha = (t-imu_t1)/(imu_t2-imu_t1);
    ai = (1-alpha)*accel_IMU.Data(tmp_idxs(1)-1,:) + alpha*accel_IMU.Data(tmp_idxs(1),:);
    
    % process update
    est_state_list(1:3,t_idx) = est_state_list(1:3,t_idx-1) + est_state_list(4:6,t_idx-1)*dt;
    est_state_list(4:6,t_idx) = est_state_list(4:6,t_idx-1) + ai'*dt;  % IMU intergration process model
      
    % process covariance 
    P = P + Q;
    
    % measurement 
    lo_vs = reshape(lo_v_ts.Data(t_idx,:),[3 4]);
    contacts = double(contact_flags.Data(t_idx,:) > 0.5)';
    sum_contacts = sum(contacts);
    
    vm = zeros(3,1);
    if (sum_contacts == 0)
        vm = zeros(3,1);
        R = diag(999*ones(3,1));
    else
        vm = lo_vs*contacts/sum_contacts;
        R = diag(0.001*ones(3,1));
    end
    
    % measurement model 
    H = zeros(3,6); H(1:3,4:6) = eye(3);
    
    y = vm - est_state_list(4:6,t_idx);
    
    
    S = H*P*H' + R;
    
    K = P*H'*inv(S);
    est_state_list(:,t_idx) = est_state_list(:,t_idx)  + K*y;
    P = (eye(6)-K*H)*P;
    est_state_time(:,t_idx) = t;
end
pos_ts = timeseries(est_state_list(1:3,:),est_state_time,'Name',"pos_ts");
vel_ts = timeseries(est_state_list(4:6,:),est_state_time,'Name',"pos_ts");
end

