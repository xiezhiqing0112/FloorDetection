file='T01_01.txt';
% path='Track3/Logfiles/processed/Training/01a-Regular/'
path='/home/mxc/wonderland/datasets/IPIN/logfiles2020/Logfiles/processed/Training/01a-Regular/';
markpoint=load(strcat(path,'P',file));
new_position=GPSToXY(markpoint(:,[2 3]));
new_position=new_position-new_position(1,:);
% plot(new_position(:,1),new_position(:,2));
% figure

markpoint(:,[2 3])=markpoint(:,[3 2]);
longitudefactor = 85000;
latitudefactor = 110000;
X=zeros(length(markpoint),1);
Y=zeros(length(markpoint),1);
for i=2:length(markpoint)
    X(i)= (markpoint(i,2)-markpoint(1,2))*longitudefactor;
    Y(i)= (markpoint(i,3)-markpoint(1,3))*latitudefactor;
end
% plot(X,Y)
markpoint(:,2)=X;
markpoint(:,3)=Y;
    % file='ipin2018/Logfiles/processed/Training/T02_02.txt'
trial_all=parseIPINdata(strcat(path,file));
startTime=zhidao_nearest(trial_all(:,2),markpoint(1,1)*1000);
startIndex=find(trial_all(:,2)==startTime);
% stopTime=zhidao_nearest(trial(:,2),markpoint(8,1)*1000);
% stopIndex=find(trial(:,2)==stopTime);
trial_all=trial_all(startIndex:length(trial_all),:);
addpath('SDK'); 
Gyroscope=trial_all(:,7:9);
Accelerometer=trial_all(:,4:6);
Magnetometer=trial_all(:,10:12);
% figure


%% ��ͨ�˲� ����alpha=0.975   ����alpha=0.67
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

gx=Gyroscope(:,1); %%����������
gy=Gyroscope(:,2); 
gz=Gyroscope(:,3); 
ax=Accelerometer(:,1); %%���ٶȼ�����
ay=Accelerometer(:,2); 
az=Accelerometer(:,3); 
mx=Magnetometer(:,1); %%��ǿ������
my=Magnetometer(:,2); 
mz=Magnetometer(:,3); 
time = trial_all(:,1) ; % each sample's collect time

%%  
% Magnititude of acceleration data from experiment data
mag = sqrt(ax.^2+ay.^2+az.^2);
% Non-gravity of acceleration
magNoG = mag - mean(mag);
% Magnititude of gyroscope data from experiment data
gyro = sqrt(gx.^2+gy.^2+gz.^2);
% Samping frequency of experiment
Fs = length(time) / (time(length(time)) - time(1))*1000;
Fs=100;
%% Low pass filter Filter
% All experiment data and reference data go through the same
% lowpass fileter

% Low pass filter
% Flp=3; % low pass filter cutofff frequency 0.8HZ
Flp=5; % low pass filter cutofff frequency 0.8HZ %����ȡ2��ã�����ȡ5���
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
gx=Gyroscope(:,1); %%����������
gy=Gyroscope(:,2); 
gz=Gyroscope(:,3); 
ax=Accelerometer(:,1); %%���ٶȼ�����
ay=Accelerometer(:,2); 
az=Accelerometer(:,3); 
mx=Magnetometer(:,1); %%��ǿ������
my=Magnetometer(:,2); 
mz=Magnetometer(:,3);

rad2deg=180/pi;
deg2rad=pi/180;
time(1)=0;
quaternion = zeros(length(time), 4);
figure
acc = sqrt(Accelerometer(:,1).^2 + Accelerometer(:,2).^2 + Accelerometer(:,3).^2);
Acc = acc-mean(acc);

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

%      StepLength = 0.6933 -0.01902*StepFreq + 0.005733*StepAV; %����ģ����ϳ����Ĳ�������
% step_sz=[0 0 0];
% step_sz=[0.6933 -0.01902 0.005733];
% Kp=0.0002;Ki=0;  
% step_sz=[ 1.456309376961421  0.074417104729315   0.290514118994777];
% Ki= -0.181743536483214;
% Kp = -0.003050424864017;

step_sz=[0.680495272569579  -0.016830682012975   0.200769775203391 ];
Ki=-0.000010868979554  ;
Kp=0.012261296315810;
%% ---------------------------- for each segment-----------------------------------------
rotation = [cos(2*0.846) -sin(2*0.846) 0;sin(2*0.846) cos(2*0.846) 0;0 0 1];
%  M =[
% 
%    -0.4862    0.8739         0;
%    -0.8739   -0.4862         0;
%          0         0         0]
% rotation=[1 0 0 ; 0 1 0; 0 0 1];
markpoint_rot=([-X Y zeros(length(markpoint(:,1)),1)]*rotation);
end_posi = [markpoint_rot(1,1) markpoint_rot(1,2)];

% plot(markpoint_rot(:,1),markpoint_rot(:,2));
startIndex=[];
stopIndex=[];
for seg_Index=1:length(markpoint)-1
    startTime=zhidao_nearest(trial_all(:,2),markpoint(seg_Index,1)*1000);
    startIndex(seg_Index)=find(trial_all(:,2)==startTime);
    stopTime=zhidao_nearest(trial_all(:,2),markpoint(seg_Index+1,1)*1000);
    stopIndex(seg_Index)=find(trial_all(:,2)==stopTime);
end
start_pose=[0 0 0];
save 'all_data.mat'
scatter(markpoint_rot(:,1),markpoint_rot(:,2),'r'); hold on;
plot(markpoint_rot(:,1),markpoint_rot(:,2),'b');
x0=[step_sz Ki Kp];
for markpoint_index=1:length(markpoint)-1
% step_cur=step_sz;
    st = startIndex(markpoint_index);
    ed = stopIndex(markpoint_index);
    delete 'all_data.mat'
    save 'all_data.mat'
%     position=segment_pos(st,ed,markpoint_index);
    x= fminunc(@segment_pos,x0);
    x0=x;  
    [positionx,positiony,end_pose]=Copy_of_recalcu_posi(x0);
    scatter(positionx,positiony,'b'); grid on; % north-up
    hold on;
      pause(0.01)
%     markpoint_index=markpoint_index+7;
end























% step_matrix=zeros(StepCount-1,3);
% YawSindeg2rad_matrix=zeros(StepCount-1,1);
% YawCosdeg2rad_matrix=zeros(StepCount-1,1);
% for k = 1:StepCount-1
%     pos_start = PeakLocation(k);
%     pos_end = PeakLocation(k+1);
% %    ftime(k)=trial(pos_start,2);
%     % orientation (yaw)
%     YawSin = mean(yaw(pos_start:pos_end,2));
%     YawCos = mean(yaw(pos_start:pos_end,3));
% %    orientation(k,1:4)=[mean(yaw(pos_start:pos_end,2)),mean(yaw(pos_start:pos_end,3)),mean(euler(pos_start:pos_end,3)),mean(euler(pos_start:pos_end,3))];
% 
%     % step length estimation
%     % SL = 0.2844 + 0.2231*frequency + 0.0426*AV
%     StepFreq = 1000/PkValue(k+1,2);
%     StepAV = var(acc(pos_start:pos_end));
%     step_matrix(k,:)=[1 -StepFreq StepAV];   
%     YawSindeg2rad_matrix(k)=(StepCount-k)* sin(YawSin*deg2rad);
%     YawCosdeg2rad_matrix(k)=(StepCount-k)* cos(YawCos*deg2rad);
% end
% distance = @(step_cur) sqrt((sum(YawSindeg2rad_matrix'*(step_cur*step_matrix')')+PositionX(1)-markpoint_rot(markpoint_index+1,1))^2+(PositionY(1)-sum(YawCosdeg2rad_matrix'*(step_cur*step_matrix')')-markpoint_rot(markpoint_index+1,2))^2);
% options = optimoptions('fminunc','Algorithm','trust-region','SpecifyObjectiveGradient',true);
% step_sz= fminunc(distance,step_sz);%,options);
