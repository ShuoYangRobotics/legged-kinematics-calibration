% assume accel_IMU, gyro_IMU, joint_ang, joint_vel, foot_force, param are available

% use IMM to determine a single velocity

num_modes = 16;    % 

% probability of each mode; initially all stance is 1 
prob = zeros(num_modes,1);prob(num_modes) = 1;
% use notations in The interacting multiple model algorithm for systems with Markovian switching coefficients


% the threshold to convert foot force to binary contact 
foot_forces_thres = [90,90,20,90];
% determing foot_forces_thres dynamically
foot_force_extrema = zeros(param.num_leg,2);

% imu acc noise 
Q0 = 0.1*eye(3);
Q = 0.1*eye(3);
% basic measurement noise 
R0 = 0.01*eye(3);
R = 0.01*eye(3);

% the probablity of transition to current state
trans_prob = 0.99;

% estimation and covariance of each mode
x = zeros(3, num_modes);
P = zeros(3,3, num_modes);
for m_idx = 1:num_modes
    P(:,:,m_idx) = 0.1*eye(3);
end
%
xout = zeros(3, 1);
Pout = zeros(3, 3);

% save output velocity timeseries 
imm_v = zeros(length(joint_vel.Data),3);

% save estimated contact 
contact_estimation = zeros(length(joint_vel.Data),4);


% for each time step 
for t_idx = 2:size(joint_vel.Time,1)
    t = joint_vel.Time(t_idx);
    dt = joint_vel.Time(t_idx) - joint_vel.Time(t_idx-1);
    
    % interpolate to get state(mocap pos and orientation)
    tmp_idxs = find(pos_mocap.Time-t>=0);
    mocap_t1 = orient_mocap.Time(tmp_idxs(1)-1);
    mocap_t2 = orient_mocap.Time(tmp_idxs(1));
    alpha = (t-mocap_t1)/(mocap_t2-mocap_t1);
    qi = quatinterp(orient_mocap.Data(tmp_idxs(1)-1,:),orient_mocap.Data(tmp_idxs(1),:), alpha,'slerp');
    R_er = quat2rotm(qi);
    
    % interpolate to get mocap vel 
    tmp_idxs = find(vel_mocap.Time-t>=0);
    mocap_t1 = vel_mocap.Time(tmp_idxs(1)-1);
    mocap_t2 = vel_mocap.Time(tmp_idxs(1));
    alpha = (t-mocap_t1)/(mocap_t2-mocap_t1);
    vi = (1-alpha)*vel_mocap.Data(tmp_idxs(1)-1,:) + alpha*vel_mocap.Data(tmp_idxs(1),:);

    % interpolate to get IMU w and a
    tmp_idxs = find(gyro_IMU.Time-t>=0);
    imu_t1 = gyro_IMU.Time(tmp_idxs(1)-1);
    imu_t2 = gyro_IMU.Time(tmp_idxs(1));
    alpha = (t-imu_t1)/(imu_t2-imu_t1);
    wi = (1-alpha)*gyro_IMU.Data(tmp_idxs(1)-1,:) + alpha*gyro_IMU.Data(tmp_idxs(1),:);
    ai = (1-alpha)*accel_IMU.Data(tmp_idxs(1)-1,:) + alpha*accel_IMU.Data(tmp_idxs(1),:);
    
    % four leg velocities
    v = zeros(3,param.num_leg);
    for i = 1:param.num_leg
        angle = joint_ang.Data(t_idx,(i-1)*3+1:(i-1)*3+3)';
        av = joint_vel.Data(t_idx,(i-1)*3+1:(i-1)*3+3)';
        p_rf = autoFunc_fk_pf_pos(angle,param.rho_opt_true(:,i),param.rho_fix(:,i));
        J_rf = autoFunc_d_fk_dt(angle,param.rho_opt_true(:,i),param.rho_fix(:,i));
        df_drho = autoFunc_d_fk_drho(angle,param.rho_opt_true(:,i),param.rho_fix(:,i));
        % it seems velocity on y direction cannot be very correctly infered 
        leg_v = (-J_rf*av-skew(wi)*p_rf);
%         leg_v = (-(J_rf*av+df_drho*drho)-skew(wi)*p_rf);
        v(:,i) = R_er*leg_v;
    end

    % from foot force, generate transition probablitity
    foot_forces = foot_force.Data(t_idx,:);
    contact_flags = foot_forces > foot_forces_thres;
    contact_flags = zeros(param.num_leg,1);
    for j=1:param.num_leg
        if (foot_forces(j) < foot_force_extrema(j,1))
            foot_force_extrema(j,1) = foot_forces(j);
        end
        if (foot_forces(j) > foot_force_extrema(j,2))
            foot_force_extrema(j,2) = foot_forces(j);
        end
        threadhold = 0.5*(foot_force_extrema(j,2)-foot_force_extrema(j,1))+foot_force_extrema(j,1);
        if (foot_forces(j) > threadhold)
            contact_flags(j) = 1;
        end
    end
    
    % convert binary contact list to contact mode
    flag_mode = bi2de(contact_flags') + 1;  % range (1-12)
    % transition matrix 
    H = (1-trans_prob)/(num_modes-1)*ones(num_modes, num_modes);
    H(flag_mode,:) = trans_prob;
    
    prob_bar = H * prob;
    
    
    %enlarge Q and R for large foot force variance 
    var_forces = zeros(1,param.num_leg);
    if t_idx > 10
        var_forces = var(foot_force.Data(t_idx-9:t_idx,:));
    end
%     Q = Q0 * (1 + sum(var_forces));
    % mix x and P
    x_i = zeros(3, num_modes);
    P_i = zeros(3,3, num_modes);
    for m_idx = 1:num_modes
        sum_j = x * (H(m_idx,:)' .* prob);
        x_i(:,m_idx) = sum_j / prob_bar(m_idx);
    end
    for m_idx = 1:num_modes
        tmp_prob = H(m_idx,:)' .* prob;
        tmp_P = zeros(3,3);
        for m_idx2 = 1:num_modes
            tmp_P = tmp_P + tmp_prob(m_idx2) * ...
                (P(:,:,m_idx2) + (x(:,m_idx2) - x_i(:,m_idx))*(x(:,m_idx2) - x_i(:,m_idx))');
        end
        P_i(:,:, m_idx) = tmp_P ./prob_bar(m_idx);
    end
    
    % update x_i and P_i 
    % calculate residual 
    residual = zeros(3, num_modes);
    if t_idx == 3000
        display(t_idx);
    end
    for m_idx = 1:num_modes
        x_i(:,m_idx) = x_i(:,m_idx) + ai'*dt;  % IMU intergration process model
%         x_i(:,m_idx) = vi;  % IMU intergration process model
        P_i(:,:, m_idx) = P_i(:,:, m_idx) + Q;
        
        % calculate residual (Innovation)
        binary_contact = de2bi(m_idx-1,param.num_leg)';
        num_contacts = sum(binary_contact)+0.0001;
        if (sum(binary_contact) == 0)
            residual(:,m_idx) =  zeros(3,1);
        else
            residual(:,m_idx) = v * binary_contact / num_contacts - x_i(:,m_idx);
        end
        % Innovation covariance
        %enlarge Q and R for large foot force variance 
        R = R0 * (1 + 10*sum(var_forces'.*binary_contact));
        S = P_i(:,:, m_idx) + R / num_contacts;
        Sinv = inv(S);
        % kalman gain 
        K = P_i(:,:, m_idx) * Sinv;
        
        % update mode prob , likelihood
        prob(m_idx) = prob_bar(m_idx) * 1/sqrt(2*pi*norm(S))*exp(-0.5*residual(:,m_idx)'* Sinv*residual(:,m_idx));
        
        % update estimation 
        x(:,m_idx) = x_i(:,m_idx) + K * residual(:,m_idx);
        P(:,:, m_idx) =  (eye(3) - K)*P_i(:,:, m_idx);
    end
    

    
    % normalize likelihood
    prob = prob / sum(prob);
    
    % final x and P 
    xout = x * prob;
    Pout = zeros(3, 3);
    for m_idx = 1:num_modes
       Pout = Pout + prob(m_idx) * (P(:,:, m_idx) + (x(:, m_idx) - xout)*(x(:, m_idx) - xout)');
    end
    
    imm_v(t_idx,:) = xout;
    
    % combine prob to find out contact probablity
    contact_estimation(t_idx,:) = zeros(1,4);
    for m_idx = 1:num_modes
        binary_contact = de2bi(m_idx-1,param.num_leg);
        contact_estimation(t_idx,:) = contact_estimation(t_idx,:) + prob(m_idx)*binary_contact;
    end
end

vel_imm = timeseries(imm_v,joint_vel.Time,'Name',"IMM velocity"); 
contact_imm = timeseries(contact_estimation,joint_vel.Time,'Name',"IMM contact");     

%% compare with vel_mocap
figure(1); clf
plot(vel_imm.Time, vel_imm.Data(:,1)) ; hold on;
% plot(vel_imm.Time, movmean(vel_imm.Data(:,1),30,1)) ; hold on;
plot(vel_mocap.Time, vel_mocap.Data(:,1)) ;
legend('vel imm X', 'vel mocap X');