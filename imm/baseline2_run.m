% assume accel_IMU, gyro_IMU, joint_ang, joint_vel, foot_force, param are available

% use baseline to determine a single velocity, use contact flag to combine leg velocities

% the threshold to convert foot force to binary contact 
foot_forces_thres = [90,90,20,90];
% determing foot_forces_thres dynamically
foot_force_extrema = zeros(param.num_leg,2);

V_N_MAX = 500;
V_N_MIN = 0.0001;
V_N_TERM2_VAR_RESCALE = 1.0e-1;
V_N_TERM3_DISTANCE_RESCALE = 1.0e-3;

% save output velocity timeseries 
baseline_v = zeros(length(joint_vel.Data),3);

% save estimated contact 
contact_estimation = zeros(length(joint_vel.Data),4);

% xref, intergration of acceleration 
xref = zeros(3,1);

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
    
    xref = xref + ai'*dt;
    
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
    
    var_forces = zeros(1,param.num_leg);
    if t_idx > 10
        var_forces = var(foot_force.Data(t_idx-9:t_idx,:));
    end
    
    % from contact flag to weight formula
    uncertainties = zeros(3,param.num_leg);
    weights = zeros(3,param.num_leg);
    for j=1:param.num_leg
        n1 = V_N_MAX*(1-contact_flags(j))+V_N_MIN;
        n2 = V_N_TERM2_VAR_RESCALE*var_forces(j);
%         n3 = V_N_TERM3_DISTANCE_RESCALE*(v(:,j) - xref) .^2 ;
%         uncertainties(:,j) = n1+n2+n3;
        uncertainties(:,j) = n1+n2;
        
        weights(:,j) = (V_N_MAX + V_N_TERM2_VAR_RESCALE + V_N_TERM3_DISTANCE_RESCALE) /  uncertainties(:,j);
        for k=1:3
            if (weights(k,j) < 0.00001) 
                weights(k,j) = 0.00001;
            end
        end
    end
    total_weights = sum(weights,2);
    
    baseline_v(t_idx,:) = zeros(1,3);
    for j=1:param.num_leg
        baseline_v(t_idx,:) = baseline_v(t_idx,:) + v(:,j)' .*weights(:,j)';
    end
    baseline_v(t_idx,:) = baseline_v(t_idx,:) ./ total_weights';
end

vel_baseline2 = timeseries(baseline_v,joint_vel.Time,'Name',"Baseline velocity");   

%% compare with vel_mocap
figure(1); clf
% plot(vel_baseline1.Time, vel_baseline1.Data(:,1), 'LineWidth',3) ; hold on;
plot(vel_baseline2.Time, vel_baseline2.Data(:,1), 'LineWidth',3) ; hold on;
% plot(vel_imm.Time, movmean(vel_imm.Data(:,1),30,1)) ; hold on;
% plot(vel_imm.Time, vel_imm.Data(:,1), 'LineWidth',3) ; hold on;
plot(vel_mocap.Time, vel_mocap.Data(:,1), 'LineWidth',3) ;
legend('estimation baseline 2',  'ground truth ');
% legend('baseline1 vel X', 'baseline2 vel X','IMM vel X',  'ground truth vel X');

%% RMSE 
[ts1 ts2] = synchronize(vel_baseline1,vel_mocap,'Intersection');
sqrt(  sum((ts1.Data(:,1) - ts2.Data(:,1)) .^2) / length(ts2.Data(:,1)))
[ts1 ts2] = synchronize(vel_baseline2,vel_mocap,'Intersection');
sqrt(  sum((ts1.Data(:,1) - ts2.Data(:,1)) .^2) / length(ts2.Data(:,1)))
[ts1 ts2] = synchronize(vel_imm,vel_mocap,'Intersection');
sqrt(  sum((ts1.Data(:,1) - ts2.Data(:,1)) .^2) / length(ts2.Data(:,1)))

