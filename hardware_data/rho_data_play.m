% live update
rho_param = cali_rho_param;

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
TOTAL_TIME = param.data_duration-1;
figure(12)
while (1)
   pause(0.001);
   time = rostime("now");
   curr_time  = time.Sec + 10^-9*time.Nsec;
   dt = curr_time - prev_time;
   elapsed_time = curr_time - init_time;
   prev_time = curr_time;
   disp([dt, elapsed_time]);
   
   tsout1 = resample(rho_param,plot_start_time:0.01:elapsed_time);
   tsout1.Data = movmean(tsout1.Data(:,[1,2]),30);
   tsout1_Time = tsout1.Time;
   tsout11_Data = tsout1.Data(:,1);
   tsout12_Data = tsout1.Data(:,2);
   
   
    tsout2 = resample(pos_mocap,plot_start_time:0.01:elapsed_time);
    tsout2_Time = tsout2.Time;
    tsout2_Data = tsout2.Data(:,1);
    tsout3 = resample(pos_est_ts1,plot_start_time:0.01:elapsed_time);
    tsout3_Time = tsout3.Time;
    tsout3_Data = tsout3.Data(1,:);
    tsout4 = resample(pos_est_ts2,plot_start_time:0.01:elapsed_time);
    tsout4_Time = tsout4.Time;
    tsout4_Data = tsout4.Data(1,:);
    
    
    tsout5 = resample(vel_mocap,plot_start_time:0.01:elapsed_time);
    tsout5_Time = tsout5.Time;
    tsout5_Data = tsout5.Data(:,1);
    tsout6 = resample(vel_est_ts1,plot_start_time:0.01:elapsed_time);
    tsout6_Time = tsout6.Time;
    tsout6_Data = tsout6.Data(1,:);
    tsout7 = resample(vel_est_ts2,plot_start_time:0.01:elapsed_time);
    tsout7_Time = tsout7.Time;
    tsout7_Data = tsout7.Data(1,:);
   refreshdata;
   
   if (elapsed_time>TOTAL_TIME)
       break;
   end
end     

clear('node')