clc
close all
clear variables

PLOTTER = false;
file='ACCEGYROMAGNAHRS.csv';
path='E:\PDR\IPIN\IPINDATA\Logfiles\01-Training\01a-Regular\T01_01/';
file_pos = 'POSI.csv';
markpoint=importdata(strcat(path,file_pos));
markpoint = markpoint.data;
trial = importdata(strcat(path,file));
trial =trial.data;
startTime=zhidao_nearest(trial(:,2),markpoint(1,1));
startIndex=find(trial(:,2)==startTime)-1000;
calib_index = 1:7000;
noise_index = 7001:startIndex;
walk_index = startIndex:length(trial);
Gyroscope=trial(:,10:12);
Accelerometer=trial(:,4:6);
Magnetometer=trial(:,16:18);
Euler = trial(:,22:24);
%% -------- PLOTTER: shs raw imu sample
if PLOTTER
    figure
        subplot(3,1,1)    
        stackedplot(Accelerometer(walk_index,:))
        title('raw imu data - accelerometer')
        subplot(3,1,2)    
        stackedplot(Gyroscope(walk_index,:))
        title('raw imu data - gyroscope')
        subplot(3,1,3)
        stackedplot(Magnetometer(walk_index,:))
        title('raw imu data - magnetometer')
    sgtitle('SHS sample raw IMU data')
    set(gcf,'position',[ 1434    138    548   489])
end

%% -------- PLOTTER: shs magnetic north raw data 
% if PLOTTER
%     figure
%     sgtitle('magnetic north sample raw IMU data')
%         subplot(3,1,1)    
%         stackedplot(target.raw_imu.accelerometer)
%         title('raw imu data - accelerometer')
%         subplot(3,1,2)    
%         stackedplot(target.raw_imu.gyroscope)
%         title('raw imu data - gyroscope')
%         subplot(3,1,3)
%         stackedplot(target.raw_imu.magnetometer)
%         title('raw imu data - magnetometer')
%     set(gcf,'position',[ 1434    138    548   489])
% end

%% ================= CALIBRATING SENSOR DATA ==========================
%% Magnetometer calibration
% clc
% disp('Calibrating magnetometer sensor data')
% 
% mag_raw = Magnetometer(calib_index,:);
% 
% N=size(mag_raw,1);
% M=ones(N,13);
% for i=1:N
%     M(i,1:9)=kron(mag_raw(i,:),mag_raw(i,:));
%     M(i,10:12)=mag_raw(i,:);
% end
% 
% cvx_begin
%     variable A(3,3)
%     variable b(3,1)
%     variable c(1,1)
%     minimize(norm( M * [vec(A) ; b ; c] , 2 ) )
%     subject to
%     trace(A) == 1
%     A-0.0001*eye(3) == semidefinite(3)
% cvx_end
% 
% invDT_invD = inv(0.25 * b' *inv(A) * b - c) *A;
% invD = chol(invDT_invD);
% bias = -0.5* inv(A) * b ;
% 
% mag_bias = bias;
% mag_invD = invD;
% 
% calib_mag = mag_invD*(Magnetometer(walk_index,:)'- mag_bias);
% Time_mag = datetime(trial(walk_index,15), 'ConvertFrom', 'posixtime' ,'TimeZone', 'local','Format','HH:mm:ss.SSS');
% calib_mag_data = timetable(Time_mag);
% calib_mag_data.X = calib_mag(:,1);
% calib_mag_data.Y = calib_mag(:,2);
% calib_mag_data.Z = calib_mag(:,3);
% 
% % Magnetic North Calibration
calib_mag_north =Magnetometer(walk_index,:);
% Time_mag_north = datetime(trial(calib_index,15), 'ConvertFrom', 'posixtime' ,'TimeZone', 'local','Format','HH:mm:ss.SSS');
calib_mag_data = timetable('SampleRate',200);
calib_mag_data.X = calib_mag_north(:,1);
calib_mag_data.Y = calib_mag_north(:,2);
calib_mag_data.Z = calib_mag_north(:,3);

%% -------- PLOTTER: magnetometer calibration
% magnetometer calib data calibration
% calib_mag_calib = mag_invD*(mag_raw'- mag_bias);
% 
% figure()
% hold on
% scatter3(calib_mag_calib(1,:),calib_mag_calib(2,:),calib_mag_calib(3,:),'r')
% scatter3(calib_mag(1,:),calib_mag(2,:),calib_mag(3,:),'b')
% scatter3(calib_mag_north(1,:),calib_mag_north(2,:),calib_mag_north(3,:),'g')
% hold off
% xlim([-1,1])
% ylim([-1,1])
% zlim([-1,1])
% legend('calibrated calibration data','calibrated SHS data','calibrated magnetic north')
% title('Calibrated Magnetometer Data')

%% Accelerometer Calibration

% acc_raw = Accelerometer(calib_index,:);
% 
% N = size(acc_raw,1);
% M=ones(N,13);
% 
% for i=1:N
%     M(i,1:9)=kron(acc_raw(i,:),acc_raw(i,:));
%     M(i,10:12)=acc_raw(i,:);
% end
% 
% cvx_begin
%     variable A(3,3)
%     variable b(3,1)
%     variable c(1,1)
%     minimize( norm( M * [vec(A) ; b ; c] , 2 ) )
%     subject to
%     trace(A) == 1
%     A-0.0001*eye(3) == semidefinite(3)
% cvx_end
% 
% invDT_invD = inv(0.25 * ( b' / A * b ) - c) * A;
% acc_invD = chol(invDT_invD);
% acc_bias = -0.5* ( A \ b );
% 
calib_acc = Accelerometer(walk_index,:);
% calib_acc = calib_acc.* 9.81;
% Time_acc = datetime(trial(walk_index,3), 'ConvertFrom', 'posixtime' ,'TimeZone', 'local','Format','HH:mm:ss.SSS');
calib_acc_data = timetable('SampleRate',200);
calib_acc_data.X = calib_acc(:,1);
calib_acc_data.Y = calib_acc(:,2);
calib_acc_data.Z = calib_acc(:,3);

%% -------- PLOTTER: accelerometer calibration

% % magnetometer calib data calibration
% calib_acc_calib = 9.81*acc_invD*(acc_raw'- acc_bias);
% 
% figure()
% hold on
% scatter3(calib_acc_calib(1,:),calib_acc_calib(2,:),calib_acc_calib(3,:),'r')
% scatter3(calib_acc(1,:),calib_acc(2,:),calib_acc(3,:),'b')
% hold off
% % xlim([-1,1])
% % ylim([-1,1])
% % zlim([-1,1])
% legend('calibrated calibration data','calibrated SHS data')
% title('Calibrated Accelerometer Data')

%% Gyroscope Calibration
gyr_bias = mean(Gyroscope(noise_index,:));
calib_gyr = Gyroscope(walk_index,:)-gyr_bias;
% Time_gyr = datetime(trial(walk_index,9), 'ConvertFrom', 'posixtime' ,'TimeZone', 'local','Format','HH:mm:ss.SSS');
calib_gyr_data = timetable('SampleRate',200);
calib_gyr_data.X = calib_gyr(:,1);
calib_gyr_data.Y = calib_gyr(:,2);
calib_gyr_data.Z = calib_gyr(:,3);

% Align sensors data
acc_data = calib_acc_data;
gyro_data = calib_gyr_data;
mag_data = calib_mag_data;
dev_comp_attitude = trial(walk_index,22:24);


%% finding variance orientation

noise_calib_mag =  Magnetometer(noise_index,:);
noise_mag_data = timetable('SampleRate',200);
noise_mag_data.X = noise_calib_mag(:,1);
noise_mag_data.Y = noise_calib_mag(:,2);
noise_mag_data.Z = noise_calib_mag(:,3);

% Accelerometer Calibration
noise_calib_acc = Accelerometer(noise_index,:);
% noise_calib_acc = noise_calib_acc.* 9.81;
noise_acc_data = timetable('SampleRate',200);
noise_acc_data.X = noise_calib_acc(:,1);
noise_acc_data.Y = noise_calib_acc(:,2);
noise_acc_data.Z = noise_calib_acc(:,3);

% Gyroscope Calibration
noise_calib_gyr = Gyroscope(noise_index,:);
noise_gyro_data = timetable('SampleRate',200);
noise_gyro_data.X = noise_calib_gyr(:,1);
noise_gyro_data.Y = noise_calib_gyr(:,2);
noise_gyro_data.Z = noise_calib_gyr(:,3);  
                 
variance.acc = max(var(noise_acc_data{:,:}));
variance.mag = max(var(noise_mag_data{:,:}));
variance.gyr = max(var(noise_gyro_data{:,:}));

%% -------- PLOTTER: noise realizations

if PLOTTER
    figure
    sgtitle('noise realization')
        subplot(3,1,1)    
        stackedplot(noise_acc_data)
        title('raw imu data - accelerometer')
        subplot(3,1,2)
        stackedplot(noise_gyro_data)
        title('raw imu data - gyroscope')
        subplot(3,1,3)
        stackedplot(noise_mag_data)
        title('raw imu data - magnetometer')
    set(gcf,'position',[ 1434    138    548   489])
end

%% Extended Kalman Filter


% Time_acc = datetime(trial(walk_index,3), 'ConvertFrom', 'posixtime' ,'TimeZone', 'local','Format','HH:mm:ss.SSS');
% calib_acc_data = timetable(Time_acc);
% calib_acc_data.X = acc_data(:,1);
% calib_acc_data.Y = acc_data(:,2);
% calib_acc_data.Z = acc_data(:,3);
% Time_mag = datetime(trial(walk_index,15), 'ConvertFrom', 'posixtime' ,'TimeZone', 'local','Format','HH:mm:ss.SSS');
% calib_mag_data = timetable(Time_mag);
% calib_mag_data.X = mag_data(:,1);
% calib_mag_data.Y = mag_data(:,2);
% calib_mag_data.Z = mag_data(:,3);
% 
% combined_raw = [acc_data; mag_data; gyro_data];
% combined_raw = sortrows(combined_raw);
%  Types = categorical({'GYR','ACC', 'MAG'});
 
 %  prior estimate
prior_est = [1,0,0,0]';
% prior_est = [dev_comp_attitude{4,:}]';

est = prior_est;

Types = categorical({'GYR','ACC', 'MAG'});

gyro_data.Type(:) = Types(1);
acc_data.Type(:) = Types(2);
mag_data.Type(:) = Types(3);

% combined_raw = [acc_data; mag_data; gyro_data];
combined_raw = [acc_data;gyro_data];
combined_raw = sortrows(combined_raw);

P = 1 * eye(4);

calAcc_R = 0.9e-3 * eye(3);

calGyr_R = 0.001 * eye(3);

calMag_R = variance.mag * eye(3);

combined = [combined_raw.X, combined_raw.Y, combined_raw.Z]';
type = [combined_raw.Type];

dT = [0; seconds(diff(gyro_data.Time))];
Time = combined_raw.Time;


estimate = repmat(struct('Time', nan, ...
                         'est',nan, ...
                         'P', nan,...
                         'error', nan,...
                         'type', nan,...
                         'corrected', nan),...
                         height(combined_raw), 1 );


g = [0; 0; -9.81];

mag_vector =  mean(Magnetometer(calib_index,:));
mag_field =  transpose(mag_vector./norm(mag_vector));

counter = 0;
progress_before= 0;
for index = 1:1:height(combined_raw)
    
    progress = round(index/height(combined_raw)*100,2);
    if mod(progress,10) == 0  && round(progress) ~= 0
        if progress~=progress_before
        disp(['percentage complete: ',  num2str(progress)]);
        progress_before = progress;
        end
    end
    
    y = combined(:,index) ;
    error = nan;
    corrected = 0;
    switch type(index)
        
        case Types(1)
            counter = counter +1;
            
            % -------------  MOTION UPDATE -----------------------
            F = eye(4) + 0.5*(dT(counter)* Somega(y));
            Gu= dT(counter)./ 2 *Sq(est);
            est = F* est; 
            J = (1/norm(est)^3)*(eye(4)*(est'*est) - (est*est'));
            est = est / norm(est);
            P = F*P*F' + Gu*calGyr_R*Gu';
%             P = J*P*J';
    % -------------- MEASUREMENT UPDATES ------------------
       
            
        case Types(2)
            % Accelerometer measurement update
            dRdq_acc = dRqdq(est);
            H_acc = [-dRdq_acc(:,:,1)'*g,...
                     -dRdq_acc(:,:,2)'*g,...
                     -dRdq_acc(:,:,3)'*g,...
                     -dRdq_acc(:,:,4)'*g];

            H = [H_acc];

            y_hat = [-quat2rotmat(est)' * g];

            R = [calAcc_R];

            error = y - y_hat;
            
            if norm(y) < 10.0 && norm(y) > 9.6
                alfa = abs(norm(y)-9.81)/0.2 + 1;
                [est,P] = measUpdate(est,P,error,H,R);            
                J = (1/norm(est)^3)*(eye(4)*(est'*est) - (est*est'));
                est = est/norm(est); 
                corrected = 1;
%                 P = J*P*J';
            end
            
         case Types(3)
             % magnetometer measurement update
            dRdq_mag = dRqdq(est);
            H_mag = [dRdq_mag(:,:,1)'*mag_field, ...
                     dRdq_mag(:,:,2)'*mag_field, ...
                     dRdq_mag(:,:,3)'*mag_field, ...
                     dRdq_mag(:,:,4)'*mag_field];

            H = [ H_mag];

            y_hat = [ quat2rotmat(est)' * mag_field];

            R = [ calMag_R];

            error = y - y_hat;
            
             if norm(y) < 1.2 && norm(y) > 0.8
                 [est,P] = measUpdate(est,P,error,H,R);
                 J = (1/norm(est)^3)*(eye(4)*(est'*est) - (est*est'));
                 est = est/norm(est);
                 corrected = 1;
%                  P = J*P*J';
             end

    end
        
%     --------------- SAVING ESTIMATE COMPONENTS ---------
    x.Time =Time(index);
    x.est = est;
    x.P = P;
    x.error = error;
    x.type = type(index);
    x.corrected = corrected;

    
    estimate(index) = x;
    
end

estimate = struct2table(estimate);
% estimate.Time = seconds(estimate.Time);
ekf_estimate = table2timetable(estimate);


%% Error plotting

% ekf_acc_errors  = ekf_estimate(ekf_estimate.type=='ACC',:);
% ekf_mag_errors  = ekf_estimate(ekf_estimate.type=='MAG',:);
% 
% if PLOTTER
%     figure
%     sgtitle('EKF errors')
%         subplot(2,1,1)    
%         stackedplot([ekf_acc_errors.error{:,:}]','DisplayLabels',{'x' 'y' 'z'})
%         title('raw imu data - accelerometer')
%         subplot(2,1,2)
%         stackedplot([ekf_mag_errors.error{:,:}]','DisplayLabels',{'x' 'y' 'z'})
%         title('raw imu data - magnetometer')
%     set(gcf,'position',[ 1434    138    548   489])
% end

%% multiplicative extended kalman filter

% mekf_estimate = MultiplicativeExtendedKalmanFilter_series(prior_est, ...
%                                 acc_data, gyro_data, mag_data, ...
%                                 calib_mag_north_data, variance, false);

%% Error plotting
% mekf_acc_errors  = mekf_estimate(ekf_estimate.type=='ACC',:);
% mekf_mag_errors  = mekf_estimate(ekf_estimate.type=='MAG',:);
% 
% if PLOTTER
%     figure
%     sgtitle('EKF errors')
%         subplot(2,1,1)    
%         stackedplot([mekf_acc_errors.error{:,:}]')
%         title('raw imu data - accelerometer')
%         subplot(2,1,2)
%         stackedplot([mekf_mag_errors.error{:,:}]')
%         title('raw imu data - magnetometer')
%     set(gcf,'position',[ 1434    138    548   489])
% end
%%
% mekf_euler_angles = timetable(mekf_estimate.Time);
% euler = q2euler([mekf_estimate.est{:,:}]);
% 
% mekf_euler_angles.yaw = euler(1,:)';
% mekf_euler_angles.pitch = euler(2,:)';
% mekf_euler_angles.roll = euler(3,:)';

%% 
ekf_euler_angles = timetable(ekf_estimate.Time);
euler = q2euler([ekf_estimate.est{:,:}]);

ekf_euler_angles.yaw = euler(1,:)';
ekf_euler_angles.pitch = euler(2,:)';
ekf_euler_angles.roll = euler(3,:)';
% 
phone_estimate = timetable('SampleRate',200);
% euler = q2euler([shs_sample.device_computed.attitude{:,:}']);

phone_estimate.yaw = deg2rad(dev_comp_attitude(:,3));
phone_estimate.pitch = deg2rad(dev_comp_attitude(:,1));
phone_estimate.roll = deg2rad(dev_comp_attitude(:,2));

% figure
% subplot(2,1,1)
% stackedplot(ekf_euler_angles)
% title('EKF estimate')
% subplot(2,1,2)
% stackedplot(phone_estimate)
% title('Phone Estimate')
% set(gcf,'position',[ 1434    138    548   489])


% figure
% subplot(3,1,1)
% stackedplot(mekf_euler_angles)
% title('EKF estimate')
% subplot(3,1,2)
% stackedplot(ekf_euler_angles)
% title('EKF estimate')
% subplot(3,1,3)
% stackedplot(phone_estimate)
% title('Phone Estimate')
% set(gcf,'position',[ 1434    138    548   489])
% 
% subplot(3,3,2)
% plot(ekf_euler_angles.Time,ekf_euler_angles.roll)
% title('EKF: roll')
% subplot(3,3,5)
% plot(mekf_euler_angles.Time,mekf_euler_angles.roll)
% title('MEKF: roll')
% subplot(3,3,8)
% plot(phone_estimate.Time,phone_estimate.roll)
% title('Phone Estimate: roll')
% 
% subplot(3,3,3)
% plot(ekf_euler_angles.Time,ekf_euler_angles.pitch)
% title('EKF: pitch')
% subplot(3,3,6)
% plot(mekf_euler_angles.Time,mekf_euler_angles.pitch)
% title('MEKF: pitch')
% subplot(3,3,9)
% plot(phone_estimate.Time,phone_estimate.pitch)
% title('Phone Estimate: pitch')

%%
% shot =-(phone_estimate.yaw - phone_estimate.yaw(1));
% long = ekf_euler_angles.yaw(1:2:end);
% for i=2:length(shot)
%     if shot(i)- shot(i-1) > pi/2
%         shot(i:end) = shot(i:end) - 2*pi;
%     elseif shot(i)- shot(i-1) < -pi/2
%         shot(i:end) = shot(i:end) + 2*pi;
%     end
% end
% for i=2:length(long)
%     if long(i)- long(i-1) > pi/2
%         long(i:end) = long(i:end) - 2*pi;
%     elseif long(i)- long(i-1) < -pi/2
%         long(i:end) = long(i:end) + 2*pi;
%     end
% end
% figure
% hold on
% plot(phone_estimate.Time,rad2deg(shot))
% plot(phone_estimate.Time,rad2deg(long))
%% 

% figure
% % subplot(3,1,1)
% hold on
% plot(ekf_euler_angles.Time, ekf_euler_angles.yaw)
% plot(phone_estimate.Time, phone_estimate.yaw)
% hold off
% legend('ekf','phone estimate')
% title('yaw')
% subplot(3,1,2)
% hold on
% plot(ekf_euler_angles.Time, ekf_euler_angles.roll)
% plot(phone_estimate.Time, phone_estimate.roll)
% hold off
% legend('ekf','phone estimate')
% title('roll')
% subplot(3,1,3)
% hold on
% plot(ekf_euler_angles.Time, ekf_euler_angles.pitch)
% plot(phone_estimate.Time, phone_estimate.pitch)
% hold off
% legend('ekf','phone estimate')
% title('pitch')
% sgtitle([ data_set_name ' example'])
%%

[shs.steps, shs.data, shs.sd_components] = stepDetection(acc_data, 'data' , false);

% Step length estimation
shs.sl_components = timetable(shs.steps.data.Time);
shs.sl_components.period = [0; seconds(diff(shs.steps.data.Time))];
shs.sl_components.period(shs.sl_components.period == 0) = nan;
shs.sl_components.freq = 1./shs.sl_components.period;

male.k1 = 0.4;
male.k = 0.3116;

test_height = 1.78;

shs.steps.data.step_length = test_height.*male.k.*sqrt(shs.sl_components.freq);
shs.steps.data.step_length(1) = test_height.*male.k1;
shs.est_distance = sum(shs.steps.data.step_length);
%
% door_handle_use = ReferenceFile2Timetable('datasets/marie testing/lopen1.2/lopen1_2/references.txt');

ekf_pred = timetable(ekf_estimate.Time);
ekf_pred.est = [ekf_estimate.est{:,:}]';
[og_positions1, step_orient1] = plotTrajectory(ekf_pred,shs);

% clear target
phone_pred = timetable(phone_estimate.Time);
phone_pred.est = angle2quat(phone_estimate.yaw,phone_estimate.pitch,phone_estimate.roll);
phone_pred = retime(phone_pred,unique(phone_estimate.Time),'linear');
[og_positions1, step_orient1] = plotTrajectory(phone_pred,shs);



%% Functions

function [est,P] = measUpdate(est,P,error,H,R)
    % EKF measurement update
    S = H*P*H' + R;
    K = (P*H') / S;
    P = P - K*S*K'; 
    est = est + K*error;
end

function R = quat2rotmat(q)
    % Convert a quaternion to a rotation matrix
    q0=q(1);   q1=q(2);   q2=q(3);   q3=q(4);
    R = [2*(q0^2+q1^2) - 1  2*(q1*q2-q0*q3)    2*(q1*q3+q0*q2);
        2*(q1*q2+q0*q3)    2*(q0^2+q2^2) - 1  2*(q2*q3-q0*q1);
        2*(q1*q3-q0*q2)    2*(q2*q3+q0*q1)    2*(q0^2+q3^2) - 1];
end

function dRdq = dRqdq(q)
    % Derivative of a rotation matrix wrt a quaternion
   q0=q(1);   q1=q(2);   q2=q(3);   q3=q(4);
   dRdq(:,:,1) = 2* [2*q0   -q3    q2;
              q3  2*q0   -q1;
             -q2    q1  2*q0];
   dRdq(:,:,2) = 2* [2*q1    q2    q3;
              q2     0   -q0;
              q3    q0     0];
   dRdq(:,:,3) = 2* [   0    q1    q0;
              q1  2*q2    q3;
             -q0    q3     0];
   dRdq(:,:,4) = 2* [   0   -q0    q1;
              q0     0    q2;
              q1    q2  2*q3];
end

function S=Somega(w)
% The matrix S(omega) defined in (13.11b)
   wx=w(1);   wy=w(2);   wz=w(3);
   S=[ 0  -wx  -wy  -wz;
      wx    0   wz  -wy;
      wy  -wz    0   wx;
      wz   wy  -wx    0];
end

function S=Sq(q)
% The matrix S(q) defined in (13.11c)
   q0=q(1);   q1=q(2);   q2=q(3);   q3=q(4);
   S=[-q1 -q2 -q3;
       q0 -q3  q2;
       q3  q0 -q1;
      -q2  q1  q0];
end


function euler = q2euler(q)
% Q2EULER  Convert quaternions to Euler angles
% euler = q2euler(q)
% q is a quaternion in columns (4xN)
% euler = [yaw(z) ; pitch(x) ; roll(y)]
  q = [q(1,:);q(3,:);q(2,:);q(4,:)];
  euler = zeros(3, size(q, 2));

  xzpwy = q(2, :).*q(4, :) + q(1, :).*q(3, :);

	IN = xzpwy+sqrt(eps)>0.5;  % Handle the north pole
  euler(1, IN) = 2*atan2(q(2, IN), q(1, IN));
  IS = xzpwy-sqrt(eps)<-0.5;  % Handle the south pole
  euler(1, IS) = -2*atan2(q(2, IS), q(1, IS));

  I = ~(IN | IS);  % Handle the default case

  euler(1, I) = atan2(2*(q(2, I).*q(3, I) - q(1, I).*q(4, I)),...
                      1-2*(q(3, I).^2 + q(4, I).^2));


  euler(3, I) = atan2(2*(q(3, I).*q(4, I) - q(1, I).*q(2, I)),...
                      1-2*(q(2, I).^2 + q(3, I).^2));

  euler(2, :) = -asin(2*xzpwy);

  euler = mod(euler+pi, 2*pi) - pi;
end
