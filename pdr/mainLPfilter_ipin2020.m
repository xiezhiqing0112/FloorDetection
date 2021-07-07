file='ACCEGYROMAGNAHRS.csv';
path='E:\PDR\IPIN\IPINDATA\Logfiles\01-Training\01a-Regular\T01_01/';
file_pos = 'POSI.csv';
trial = importdata(strcat(path,file));
trial =trial.data;
markpoint=importdata(strcat(path,file_pos));
markpoint = markpoint.data;
new_position=GPSToXY(markpoint(:,[3 4]));
new_position=new_position-new_position(1,:);
plot(new_position(:,1),new_position(:,2));
startTime=zhidao_nearest(trial(:,2),markpoint(1,1));
startIndex=find(trial(:,2)==startTime);
startIndex=11001;
trial=trial(startIndex:end,:);
%  trial = flip(trial);
addpath('SDK'); 
Gyroscope=trial(:,10:12);
Accelerometer=trial(:,4:6);
Magnetometer=trial(:,16:18);
Euler = trial(:,22:24);
Time = trial(:,2);
gx=Gyroscope(:,1); %%陀螺仪数据
gy=Gyroscope(:,2); 
gz=Gyroscope(:,3); 
ax=Accelerometer(:,1); %%加速度计数据
ay=Accelerometer(:,2); 
az=Accelerometer(:,3); 
mx=Magnetometer(:,1); %%磁强计数据
my=Magnetometer(:,2); 
mz=Magnetometer(:,3); 
time = Time ; % each sample's collect time

%%  
% Magnititude of acceleration data from experiment data
mag = sqrt(ax.^2+ay.^2+az.^2);
% Non-gravity of acceleration
magNoG = mag - mean(mag);
% Magnititude of gyroscope data from experiment data
gyro = sqrt(gx.^2+gy.^2+gz.^2);
% Samping frequency of experiment
Fs = length(Time) / (Time(length(Time)) - Time(1));
% Fs=100;

%% Low pass filter Filter
% All experiment data and reference data go through the same
% lowpass fileter

% Low pass filter
% Flp=3; % low pass filter cutofff frequency 0.8HZ
Flp=2; % low pass filter cutofff frequency 0.8HZ %室内取2最好；室外取5最好
[b,a]=butter(5,Flp*2/Fs,'low'); % Butter lowpass filter setup

% [b,a]=butter(5,0.03,'low'); % Butter lowpass filter setup
% Still failed to write these in functions
% filtfilt function to cutoff the initial bias of filter
LPFAx = filtfilt(b, a, ax); % LPF output of acceleration x-axis
LPFAy = filtfilt(b, a, ay);
LPFAz = filtfilt(b, a, az);
LPFMx = filtfilt(b, a, mx);
LPFMy = filtfilt(b, a, my);
LPFMz = filtfilt(b, a, mz);
LPFGx = filtfilt(b, a, gx);
LPFGy = filtfilt(b, a, gy);
LPFGz = filtfilt(b, a, gz);

% LPFGyro = filtfilt(b,a,gyro); % LPF output raw gyroscope data

% Accelerometer=[LPFAx,LPFAy,LPFAz];
% Gyroscope=[LPFGx,LPFGy,LPFGz];
% Magnetometer=[LPFMx,LPFMy,LPFMz];
gx=Gyroscope(:,1); %%陀螺仪数据
gy=Gyroscope(:,2); 
gz=Gyroscope(:,3); 
ax=Accelerometer(:,1); %%加速度计数据
ay=Accelerometer(:,2); 
az=Accelerometer(:,3); 
mx=Magnetometer(:,1); %%磁强计数据
my=Magnetometer(:,2); 
mz=Magnetometer(:,3);

rad2deg=180/pi;
deg2rad=pi/180;
% time(1)=0;
quaternion = zeros(length(Time), 4);
%% Process sensor data through algorithm

% 使用下标为index的样本计算初始方向
index=20;
Pitch0=asin(ax(index)/sqrt(ax(index)*ax(index)+ay(index)*ay(index)+az(index)*az(index)));%初始角计算
Roll0=atan2(-ay(index),-az(index));
Yaw0=atan2(-my(index)*cos(Roll0)+mz(index)*sin(Roll0),mx(index)*cos(Pitch0)+my(index)*sin(Pitch0)*sin(Roll0)+mz(index)*sin(Pitch0)*cos(Roll0))-8.3*pi/180;
% Yaw0*rad2deg;
[q0,q1,q2,q3]=Quaternion_FromEulerAngle(Roll0,Pitch0,Yaw0);%初始四元数计算
% [q0,q1,q2,q3]=Quaternion_FromEulerAngle(0,0,90*deg2rad);%初始四元数计算
q(1,:)=[q0,q1,q2,q3];

% q(1,:) = initQuat( Accelerometer(200,:)', Magnetometer(200,:)' );
% kalman_euler = quatToEuler(quat)*rad2deg;

Pitch(1)= Pitch0*rad2deg; 
Roll(1) =Roll0*rad2deg;
Yaw(1) = Yaw0*rad2deg;
tt(1)=0;                                      %算法单次仿真时间测量
SamplePeriod=1/Fs;                            %数据采样时间
Beta=0.009;                                    %梯度下降法梯度步长设置
Kp=0.5;Ki=0.5;                                  %叉积法比例积分系数设置
eInt(1,:)=[0 0 0];                            %叉积法误差初值
Length= size(trial,1);   
addpath('quaternion_library');      % include quaternion library

% addpath('direction')
% [ fusedOrientation ] = fuseSensors( Accelerometer, Magnetometer, Gyroscope, trial(:,1) );
%% Process sensor data through algorithm

%  AHRS = MadgwickAHRS('SamplePeriod', 1/100, 'Beta', 0.1);
AHRS = MahonyAHRS('SamplePeriod', SamplePeriod, 'Kp', 1,'Ki',1);

quaternion = zeros(length(trial), 4);
quaternion(1,:)=q(1,:);
for t = 2:length(trial)
%     AHRS.Update(Gyroscope(t,:), Accelerometer(t,:), Magnetometer(t,:),quaternion(t-1, :));	% gyroscope units must be radians
    AHRS.UpdateIMU(Gyroscope(t,:), Accelerometer(t,:),quaternion(t-1, :));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
    
end
%  Accelerometer2 = quaternRotate(Accelerometer, quaternConj(quaternion));
 
%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ?90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.
% hold on;
% plot(euler(:,3));
%%四元素求航向角 结束
%  euler(:,3)=Yaw;
for i=1:length(euler)
    if euler(i,3)<0
        euler(i,3)=euler(i,3)+360;
    end
end

yaw=euler(:,3);
% yaw=360-yaw;
yaw_save= yaw;
% hold on;
% plot(yaw2);
zoneA = find(yaw>=0 & yaw<90);
zoneB = find(yaw>=90 & yaw<180);
zoneC = find(yaw>=180 & yaw<270);
zoneD = find(yaw>=270 & yaw<360);
% range of sin -->(-90,+90)
% range of cos -->(0,+180)
yaw(zoneA,2:3) = [yaw(zoneA,1) yaw(zoneA,1)];
yaw(zoneB,2:3) = [180-yaw(zoneB,1) yaw(zoneB,1)];
yaw(zoneC,2:3) = [180-yaw(zoneC,1) 360-yaw(zoneC,1)];
yaw(zoneD,2:3) = [yaw(zoneD,1)-360 360-yaw(zoneD,1)];

flatMagnetic = quaternRotate(Magnetometer, quaternConj(quaternion));

% figure
acc = sqrt(Accelerometer(:,1).^2 + Accelerometer(:,2).^2 + Accelerometer(:,3).^2);
acc = acc-mean(acc);

%% BANDPASS FILTER
fs=100;
f1=0.75;               % cuttoff low frequency to get rid of baseline wander
f2=2.75;                 % cuttoff frequency to discard high frequency noise
Wn=[f1 f2]/(fs/2);    % cutt off frequency based on fs
N = 4;                % order of 3 less processing

[a,b] = butter(N,Wn); % bandpass filtering
bandPass = filtfilt(a,b,acc);
% bandPass = bandPass/ max(abs(bandPass));
% plot(acc)

%% find peaks
% Find signal peaks - peaks under a threshold value are considered as noise.
% [PkValue, PeakLocation] = findpeaks(acc, 'MINPEAKHEIGHT', 0.25);
% % 
% % % time interval (steps between 0.4s and 2s)
% PkValue(:,2) = trial(PeakLocation,1);
% PkValue(2:end,2) = PkValue(2:end,2) - PkValue(1:end-1,2);
% index = find(PkValue(:,2) < 400);
% if isempty(index) == 0
%     pos_del = [];
%     for k = 1:length(index)
%         temp = index(k); % position of the suspicious samples
%         pos_del = [pos_del; temp];
%         if PkValue(temp,1) <= PkValue(temp-1,1)
%             pos_del = [pos_del; temp];
%         else
%             pos_del = [pos_del; temp-1];
%         end
%     end
%     PeakLocation(pos_del) = [];
%     PkValue(pos_del,:) = [];
% end

 % step number

% %% High-pass filter direction to remove drift
% order = 1;
% filtCutOff = 0.05;
% samplePeriod=1/100;
% [b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
% yaw = filtfilt(b, a, yaw);

%% position update
[PkValue, PeakLocation] = stateStepDetection(ax, ay, az);
% PkValue = [zeros(length(PkValue),1) PkValue];
StepCount = length(PkValue);
PositionX = zeros(StepCount, 1);
PositionY = zeros(StepCount, 1);


PositionX(1)=0;
PositionY(1) = 0;

distance = 0;
StepLengthArr=[];
orientation=[];
yaw_get=[];
ftime=[];
turn_flag = [];


for k = 1:StepCount-3
    pos_start = PeakLocation(k);
    pos_end = PeakLocation(k+1);
    yaw_diff = max(euler(PeakLocation(k):PeakLocation(k+3),3)) - min(euler(PeakLocation(k):PeakLocation(k+3),3));
    if yaw_diff >65
        turn_flag = [turn_flag, k+1];
    end
    YawSin = mean(euler(pos_start:pos_end,3));
    YawCos = mean(euler(pos_start:pos_end,3));
    % step length estimation
    % SL = 0.2844 + 0.2231*frequency + 0.0426*AV
    StepFreq = 1000/(PkValue(k)*2);
    StepAV = var(acc(pos_start:pos_end));
     %StepLength = 0.2844 + 0.2231*StepFreq + 0.0426*StepAV;%开源工程原有参数
     StepLength = 0.6933 -0.01902*StepFreq + 0.005733*StepAV; %消防模块拟合出来的步长参数
    % StepLength = 4.484 -4.183*StepFreq + 0.1115*StepAV; %消防模块拟合出来的步长参数
     % StepLength = 2.799 -2.817*StepFreq + 0.2988*StepAV; %消防模块拟合出来的步长参数
%     StepLength=0.65;
    distance = distance + StepLength;
    StepLengthArr(k)=StepLength;
    % position update
%     PositionX(k+1) = PositionX(k) + StepLength * cos(deg2rad(YawCos));
%     PositionY(k+1) = PositionY(k) + StepLength * sin(deg2rad(YawSin));
%     scatter(PositionY(k+1),PositionX(k+1)); grid on; % north-up
 
    PositionY(k+1) = PositionY(k) + StepLength * sin(YawSin*deg2rad);
    PositionX(k+1) = PositionX(k) - StepLength * cos(YawCos*deg2rad);
%     scatter(PositionX(k+1),PositionY(k+1),'b'); grid on; % north-up
%     hold on;
%       pause(0.01)
end
figure
plot(PositionX(1:end-3),PositionY(1:end-3),'b'); grid on; 
% hold on 
% scatter(PositionX(turn_flag),PositionY(turn_flag),'r');
% hold on;
% plot(markpoint(:,2),markpoint(:,3));
% plot(X,Y,'r')
% distance
%  sqrt(PositionX(end)*PositionX(end)+PositionY(end)*PositionY(end))

% scatter(PositionX(idx),PositionY(idx),'r'); % north-up
% delta_yaw=[];
% 
% idx=[];
% for i=1:length(markpoint)
%     deltat = abs(ftime-markpoint(i,1)*1000);
%     idx(i)=find(deltat==min(deltat));
% end
% figure
% yaw_save = yaw_save(PeakLocation);+-1)));
% figure
% plot(delta_yaw,'r')
%  figure
%  scatter(yaw_get(:,1),yaw_get(:,2),'b')
