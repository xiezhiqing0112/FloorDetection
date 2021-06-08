% clear
% close all
% clc
% filename='beikeda.csv';
% filename='SensorLog_22117-10122018.csv';
% addpath('csv_import')
% if exist('trial.mat')
%     load trial.mat
% else
%     [ttimestamp] = csvimport( filename,'delimiter',';','noHeader', false);
%     ttimestamp=ttimestamp(2:length(ttimestamp),:);
%     ttimestamp(:,17)=[];
%     trial = cell2mat(ttimestamp);
%       save trial;
% end
% Gyroscope=trial(:,7:9);
% Accelerometer=trial(:,4:6);
% Magnetometer=trial(:,10:12);


file='T01_01.txt';
% path='Track3/Logfiles/processed/Training/01a-Regular/'
% path='/home/mxc/wonderland/datasets/IPIN/logfiles2020/Logfiles/processed/Training/01a-Regular/';
% quaternion_path = '/home/mxc/wonderland/datasets/IPIN/processed/01-Training/01a-Regular/T01_01/quaternion_6_axis_3.0.csv';
% align_file='/home/mxc/wonderland/datasets/IPIN/processed/01-Training/01a-Regular/T01_01/ACCEGYROMAGN.csv';
path='/home/mxc/wonderland/datasets/IPIN/logfiles2020/Logfiles/processed/Training/01a-Regular/';
% quaternion_path = '/home/mxc/wonderland/datasets/IPIN/cardboard_quanter.csv';

align_file='/home/mxc/wonderland/ipin_data/01-Training/01a-Regular/T01_01/ACCEGYROMAGNAHRS.csv';

markpoint=load(strcat(path,'P',file));
% figure
% plot(markpoint(:,2),markpoint(:,3));
new_position=GPSToXY(markpoint(:,[2 3]));
new_position=new_position-new_position(1,:);
% figure
% plot(new_position(:,1),new_position(:,2));
% longitudefactor = 85000;
% latitudefactor = 110000;
% X=zeros(length(markpoint),1);
% Y=zeros(length(markpoint),1);
% for i=2:length(markpoint)
%     X(i)= (markpoint(i,2)-markpoint(1,2))*longitudefactor;
%     Y(i)= (markpoint(i,3)-markpoint(1,3))*latitudefactor;
% end
% % plot(X,Y)
% markpoint(:,2)=X;
% markpoint(:,3)=Y;

markpoint(:,2:3) = new_position;
%     file='ipin2018/Logfiles/processed/Training/T02_02.txt'
 trial=csvread(align_file,4,1);
 startTime=zhidao_nearest(trial(:,1),markpoint(1,1));
 startIndex=find(trial(:,1)==startTime);
 
%  stopTime=zhidao_nearest(trial(:,2),markpoint(8,1)*1000);
%  stopIndex=find(trial(:,2)==stopTime);
%  stopIndex=0;
trial=trial(startIndex:end,:);
addpath('SDK'); 
Gyroscope=trial(:,9:11);
Accelerometer=trial(:,3:5);
Magnetometer=trial(:,15:17);
% figure



%% 低通滤波 室内alpha=0.975   室外alpha=0.67
accc=zeros(length(Accelerometer),3);
mag=zeros(length(Magnetometer),3);
gyro=zeros(length(Gyroscope),3);
alpha=0.975;

for i=2:length(Accelerometer)
    accc(i,:) = alpha * accc(i-1,:) + (1 - alpha)* Accelerometer(i,:);
     mag(i,:) = alpha * mag(i-1,:) + (1 - alpha)* Magnetometer(i,:);
     gyro(i,:) = alpha * gyro(i-1,:) + (1 - alpha)* Gyroscope(i,:);
end

%%

gx=Gyroscope(:,1); %%陀螺仪数据
gy=Gyroscope(:,2); 
gz=Gyroscope(:,3); 
ax=Accelerometer(:,1); %%加速度计数据
ay=Accelerometer(:,2); 
az=Accelerometer(:,3); 
mx=Magnetometer(:,1); %%磁强计数据
my=Magnetometer(:,2); 
mz=Magnetometer(:,3); 
time = trial(:,1) ; % each sample's collect time

%%  
% Magnititude of acceleration data from experiment data
mag = sqrt(ax.^2+ay.^2+az.^2);
% Non-gravity of acceleration
magNoG = mag - mean(mag);
% Magnititude of gyroscope data from experiment data
gyro = sqrt(gx.^2+gy.^2+gz.^2);
% Samping frequency of experiment
Fs = length(time) / (time(length(time)) - time(1))*1000;
Fs=200;
%% Low pass filter Filter
% All experiment data and reference data go through the same
% lowpass fileter

% Low pass filter
% Flp=3; % low pass filter cutofff frequency 0.8HZ
Flp=5; % low pass filter cutofff frequency 0.8HZ %室内取2最好；室外取5最好
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

LPFGyro = filtfilt(b,a,gyro); % LPF output raw gyroscope data

Accelerometer=[LPFAx,LPFAy,LPFAz];
Gyroscope=[LPFGx,LPFGy,LPFGz];
Magnetometer=[LPFMx,LPFMy,LPFMz];
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
time(1)=0;
quaternion = zeros(length(time), 4);
%% Process sensor data through algorithm
% 使用下标为index的样本计算初始方向
index=20;
Pitch0=asin(ax(index)/sqrt(ax(index)*ax(index)+ay(index)*ay(index)+az(index)*az(index)));%初始角计算
Roll0=atan2(-ay(index),-az(index));
Yaw0=atan2(-my(index)*cos(Roll0)+mz(index)*sin(Roll0),mx(index)*cos(Pitch0)+my(index)*sin(Pitch0)*sin(Roll0)+mz(index)*sin(Pitch0)*cos(Roll0))-8.3*pi/180;
Yaw0*rad2deg;
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
Kp=0.0002;Ki=0.0001;                                  %叉积法比例积分系数设置
eInt(1,:)=[0 0 0];                            %叉积法误差初值
Length= size(trial,1);   

% for t = 2:Length
%     tic;
% %     q(t,:) = madgwickAHRS(Gyroscope(t,:), Accelerometer(t,:), Magnetometer(t,:),q(t-1, :),Beta,SamplePeriod);	%梯度下降法  
%    [q(t,:),eInt(t,:)] = MahonyAHRSupdate( Gyroscope(t,:), Accelerometer(t,:), Magnetometer(t,:),q(t-1, :),SamplePeriod,Ki,Kp,eInt(t-1,:));%Mahony法
%     tt(t)=toc;%计算算法运行时间
%     
%     T=[ 1 - 2 * (q(t,4) *q(t,4) + q(t,3) * q(t,3)) 2 * (q(t,2) * q(t,3) +q(t,1) * q(t,4)) 2 * (q(t,2) * q(t,4)-q(t,1) * q(t,3));
%         2 * (q(t,2) * q(t,3)-q(t,1) * q(t,4)) 1 - 2 * (q(t,4) *q(t,4) + q(t,2) * q(t,2)) 2 * (q(t,3) * q(t,4)+q(t,1) * q(t,2));
%         2 * (q(t,2) * q(t,4) +q(t,1) * q(t,3)) 2 * (q(t,3) * q(t,4)-q(t,1) * q(t,2)) 1 - 2 * (q(t,2) *q(t,2) + q(t,3) * q(t,3))];%cnb
%     Roll(t)  = atan2(T(2,3),T(3,3))*rad2deg;
%     Pitch(t) = asin(-T(1,3))*rad2deg;
%     Yaw(t)   = atan2(T(1,2),T(1,1))*rad2deg-8.3;  
% end
% plot(Yaw);

 addpath('quaternion_library');      % include quaternion library

% addpath('direction')
% [ fusedOrientation ] = fuseSensors( Accelerometer, Magnetometer, Gyroscope, trial(:,1) );
%% Process sensor data through algorithm

%  AHRS = MadgwickAHRS('SamplePeriod', 1/100, 'Beta', 0.1);
AHRS = MahonyAHRS('SamplePeriod', SamplePeriod, 'Kp', 0.002,'Ki',0);

quaternion = zeros(length(trial), 4);
quaternion(1,:)=q(1,:);
for t = 2:length(trial)
    AHRS.Update(Gyroscope(t,:), Accelerometer(t,:), Magnetometer(t,:),quaternion(t-1, :));	% gyroscope units must be radians
%     AHRS.UpdateIMU(Gyroscope(t,:), Accelerometer(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
    
end
%  Accelerometer2 = quaternRotate(Accelerometer, quaternConj(quaternion));
 
%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ?90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock
% quaternion=csvread(quaternion_path,1,1);
% quaternion = quaternion(startIndex:length(quaternion(:,1)),:);

%% origin data
% quaternion=[ones(length(trial(:,1)),1) trial(:,24:26)];
quaternion(:,2:4)=trial(:,24:26);
%%
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
% figure
% % temp=trial(:,23);
% % for i=1:length(temp)
% %     if temp(i)<0
% %         temp(i)=temp(i)+360;
% %     end
% % end
% % plot(temp)
% % hold on; 
% % 
% % plot(yaw);
% % yaw=temp;
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

% flatMagnetic = quaternRotate(Magnetometer, quaternConj(quaternion));

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
% plot(acc,'b')

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

figure
PositionX(1)=0;
PositionY(1) = 0;

distance = 0;
StepLengthArr=[];
orientation=[];
yaw_get=[];
ftime=[];
for k = 1:StepCount-1
    pos_start = PeakLocation(k);
    pos_end = PeakLocation(k+1);
    % orientation (yaw)
    YawSin = mean(yaw(pos_start:pos_end,2));
    YawCos = mean(yaw(pos_start:pos_end,3));
    orientation(k,1:4)=[mean(yaw(pos_start:pos_end,2)),mean(yaw(pos_start:pos_end,3)),mean(euler(pos_start:pos_end,3)),mean(euler(pos_start:pos_end,3))];
%     yaw_save(k)=YawCos;
    ftime(k)=trial(pos_start,2);
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
 
    PositionX(k+1) = PositionX(k) + StepLength * sin(YawSin*deg2rad);
    PositionY(k+1) = PositionY(k) - StepLength * cos(YawCos*deg2rad);
    scatter(-PositionX(k+1),PositionY(k+1),'b'); grid on; % north-up
    hold on;
      pause(0.01)
end
plot(markpoint(:,2),markpoint(:,3));



% 
% x=[];
% y=[];
% x(1)=0;
% y(1)=0;
% for i=1:length(trial(:,1))
%     x(i+1)=x(i)+0.3*sin(trial(i,23));
%     y(i+1)=y(i)-0.3*cos(trial(i,23));
% end
% scatter(x,y,'g');