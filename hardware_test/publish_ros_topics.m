% publish following topics:
% - pos_est_ts vel_est_ts    (uncalibrated)
% - pos_est_ts2 vel_est_ts2  (calibrated)
% - rho_param
% - pos_mocap
% - vel_mocap

DlgH = figure(4);
H = uicontrol('Style', 'PushButton', ...
                    'String', 'Break', ...
                    'Callback', 'delete(gcbf)');
rosshutdown                
rosinit           
node = ros.Node('/matlab_publish');
elapsed_time = 0;
time = rostime("now");
prev_time = time.Sec + 10^-9*time.Nsec;
curr_time = time.Sec + 10^-9*time.Nsec;
init_time = time.Sec + 10^-9*time.Nsec;
TOTAL_TIME = 35;

% publish pos_mocap
pub1 = ros.Publisher(node,'/ground_truth_position','geometry_msgs/Vector3','DataFormat','struct');
msg1 = rosmessage(pub1);
% publish pos_est_ts
pub2 = ros.Publisher(node,'/estimation_position_FIX_LEG_LENGTH','geometry_msgs/Vector3','DataFormat','struct');
msg2 = rosmessage(pub2);
% publish pos_est_ts2
pub3 = ros.Publisher(node,'/estimation_position_CALIBRATED_LEG_LENGTH','geometry_msgs/Vector3','DataFormat','struct');
msg3 = rosmessage(pub3);

% publish vel_mocap
pub4 = ros.Publisher(node,'/ground_truth_velocity','geometry_msgs/Vector3','DataFormat','struct');
msg4 = rosmessage(pub4);
% publish vel_est_ts
pub5 = ros.Publisher(node,'/estimation_velocity_FIX_LEG_LENGTH','geometry_msgs/Vector3','DataFormat','struct');
msg5 = rosmessage(pub5);
% publish pos_est_ts2
pub6 = ros.Publisher(node,'/estimation_velocity_CALIBRATED_LEG_LENGTH','geometry_msgs/Vector3','DataFormat','struct');
msg6 = rosmessage(pub6);
% publish rho_param
pub7 = ros.Publisher(node,'/calf_length_leg1','std_msgs/Float64','DataFormat','struct');
msg7 = rosmessage(pub7);
pub8 = ros.Publisher(node,'/calf_length_leg2','std_msgs/Float64','DataFormat','struct');
msg8 = rosmessage(pub8);
pub9 = ros.Publisher(node,'/calf_length_leg3','std_msgs/Float64','DataFormat','struct');
msg9 = rosmessage(pub9);
pub10 = ros.Publisher(node,'/calf_length_leg4','std_msgs/Float64','DataFormat','struct');
msg10 = rosmessage(pub10);


while (ishandle(H))
   pause(0.002);
   time = rostime("now");
   curr_time  = time.Sec + 10^-9*time.Nsec;
   dt = curr_time - prev_time;
   elapsed_time = curr_time - init_time;
   prev_time = curr_time;
   disp([dt, elapsed_time]);
   
   % publish pos_mocap
   pos_instance = resample(pos_mocap,elapsed_time);
   msg1.X = pos_instance.Data(1);
   msg1.Y = pos_instance.Data(2);
   msg1.Z = pos_instance.Data(3);
   send(pub1,msg1);
   
   % publish pos_est_ts
   pos_instance = resample(pos_est_ts,elapsed_time);
   msg2.X = pos_instance.Data(1);
   msg2.Y = pos_instance.Data(2);
   msg2.Z = pos_instance.Data(3);
   send(pub2,msg2);
   
   % publish pos_est_ts2
   pos_instance = resample(pos_est_ts2,elapsed_time);
   msg3.X = pos_instance.Data(1);
   msg3.Y = pos_instance.Data(2);
   msg3.Z = pos_instance.Data(3);
   send(pub3,msg3);
   
   % publish vel_mocap
   pos_instance = resample(vel_mocap,elapsed_time);
   msg4.X = pos_instance.Data(1);
   msg4.Y = pos_instance.Data(2);
   msg4.Z = pos_instance.Data(3);
   send(pub4,msg4);
   
   % publish vel_est_ts
   pos_instance = resample(vel_est_ts,elapsed_time);
   msg5.X = pos_instance.Data(1);
   msg5.Y = pos_instance.Data(2);
   msg5.Z = pos_instance.Data(3);
   send(pub5,msg5);
   
   % publish vel_est_ts2
   pos_instance = resample(vel_est_ts2,elapsed_time);
   msg6.X = pos_instance.Data(1);
   msg6.Y = pos_instance.Data(2);
   msg6.Z = pos_instance.Data(3);
   send(pub6,msg6);
   
   % rho_param
   rho_instance = resample(rho_param,elapsed_time);
   msg7.Data = rho_instance.Data(1);
   msg8.Data = rho_instance.Data(2);
   msg9.Data = rho_instance.Data(3);
   msg10.Data = rho_instance.Data(4);
   send(pub7,msg7);
   send(pub8,msg8);
   send(pub9,msg9);
   send(pub10,msg10);
   
   if (elapsed_time>TOTAL_TIME)
       break;
   end
end     

clear('node')