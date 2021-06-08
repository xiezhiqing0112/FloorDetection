function loss = quaternion_with_cardboard(x)
    step_sz = x(1:3);
%     step_sz=[0.6933 -0.01902 0.005733];
    Ki = x(4);
    Kp = x(5);
    yaw_w=x(6:7);
    rad2deg=180/pi;
    deg2rad=pi/180;
    tt(1)=0;                                      %算法单次仿真时间测量
    Fs=100;
    SamplePeriod=1/Fs;                            %数据采样时间
    
%     Beta=0.009;                                    %梯度下降法梯度步长设置
                                    %叉积法比例积分系数设置
    file='T01_01.txt';
    % path='Track3/Logfiles/processed/Training/01a-Regular/'
    path='/home/mxc/wonderland/datasets/IPIN/logfiles2020/Logfiles/processed/Training/01a-Regular/';
    quaternion_path = '/home/mxc/wonderland/datasets/IPIN/processed/T01_01/quanternion.csv';
    acc_path = '/home/mxc/wonderland/datasets/IPIN/processed/T01_01/ACCE_align.csv';
    
    markpoint=load(strcat(path,'P',file));
    GPS_pos=markpoint(:,2:3);
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
        
    trial_all=csvread(acc_path,2,1);
    startTime=zhidao_nearest(trial_all(:,4),markpoint(1,1)*1000);
    startIndex=find(trial_all(:,4)==startTime);
    % stopTime=zhidao_nearest(trial(:,2),markpoint(8,1)*1000);
    % stopIndex=find(trial(:,2)==stopTime);
    trial_all=trial_all(startIndex:length(trial_all(:,1)),:);
    addpath('SDK'); 
    Accelerometer=trial_all(:,6:8);
    % figure


    %% 低通滤波 室内alpha=0.975   室外alpha=0.67
    accc=zeros(length(Accelerometer),3);
    alpha=0.975;

    for i=2:length(Accelerometer)
        accc(i,:) = alpha * accc(i-1,:) + (1 - alpha)* Accelerometer(i,:);
    end

    %%
    ax=Accelerometer(:,1); %%加速度计数据
    ay=Accelerometer(:,2); 
    az=Accelerometer(:,3); 
    time = trial_all(:,4) ; % each sample's collect time

    % Samping frequency of experiment
    Fs = length(time) / (time(length(time)) - time(1))*1000;
    Fs=100;
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

    Accelerometer=[LPFAx,LPFAy,LPFAz];
    ax=Accelerometer(:,1); %%加速度计数据
    ay=Accelerometer(:,2); 
    az=Accelerometer(:,3); 

    rad2deg=180/pi;
    deg2rad=pi/180;
    time(1)=0;
    quaternion = zeros(length(time), 4);
    % figure
    acc = sqrt(Accelerometer(:,1).^2 + Accelerometer(:,2).^2 + Accelerometer(:,3).^2);
    acc = acc-mean(acc);
    figure
    plot(acc)
    %% BANDPASS FILTER
    fs=100;
    f1=0.75;               % cuttoff low frequency to get rid of baseline wander
    f2=2.75;                 % cuttoff frequency to discard high frequency noise
    Wn=[f1 f2]/(fs/2);    % cutt off frequency based on fs
    N = 4;                % order of 3 less processing

    [a,b] = butter(N,Wn); % bandpass filtering
    bandPass = filtfilt(a,b,acc);
    
    startIndex=[];
    stopIndex=[];
    % for seg_Index=1:length(markpoint)-1
    %     startTime=zhidao_nearest(trial_all(:,2),markpoint(seg_Index,1)*1000);
    %     startIndex(seg_Index)=find(trial_all(:,2)==startTime);
    %     stopTime=zhidao_nearest(trial_all(:,2),markpoint(seg_Index+1,1)*1000);
    %     stopIndex(seg_Index)=find(trial_all(:,2)==stopTime);
    % end
    % x= [Ki Kp]; 
    % temp_min=[step_sz Ki Kp];
    trial=trial_all;
    % step_cur=step_sz;

    quaternion=csvread(quaternion_path,2,1);
    quaternion = quaternion(startIndex:length(quaternion(:,1)),2:4);
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
    yaw=yaw_w(1)*yaw+yaw_w(2);
    %%
%     yaw=yaw_w(1)*(yaw.^2)+yaw_w(2)*yaw+yaw_w(3);
    yaw_save=yaw;
    %%
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

    % flatMagnetic = quaternRotate([mx my mz], quaternConj(quaternion));

    [PkValue, PeakLocation] = stateStepDetection(ax,ay,az);
    StepCount=length(PkValue);
    %% position update
    PositionX = zeros(StepCount, 1);
    PositionY = zeros(StepCount, 1);
    PositionX(1)=markpoint(1,2);
    PositionY(1)=markpoint(1,3);
    ftime=[];
%     distance = 0;
    for k = 1:StepCount-1
        pos_start = PeakLocation(k);
        pos_end = PeakLocation(k+1);
    %    ftime(k)=trial(pos_start,2);
        % orientation (yaw)
        ftime(k)= trial(pos_start,2);
        YawSin = mean(yaw(pos_start:pos_end,2));
        YawCos = mean(yaw(pos_start:pos_end,3));
        
        StepFreq = 500/PkValue(k+1);
        StepAV = var(acc(pos_start:pos_end));
        
         StepLength = step_sz(1) +step_sz(2)*StepFreq + step_sz(3)*StepAV;
      
        PositionY(k+1) = PositionY(k) + StepLength * sin(YawSin*deg2rad);
        PositionX(k+1) = PositionX(k) - StepLength * cos(YawCos*deg2rad);
    end
    %% 计算误差
    idx=[];
    for i=1:length(markpoint)
        deltat = abs(ftime-markpoint(i,1)*1000);
        idx(i)=find(deltat==min(deltat));
    end
    markpoint_angle=[];
    markpoint_angle(1)=get_angle(GPS_pos(1,:),GPS_pos(1,:));
    for i=1:length(markpoint)-1
        markpoint_angle(i+1)=get_angle(GPS_pos(i,:),GPS_pos(i+1,:));
    end
    delta_markpoint=(markpoint_angle(2:end-1)-markpoint_angle(1:end-2))';
    yaw_save = yaw_save(PeakLocation);
    delta_yaw = yaw_save(idx(2:end-1))-yaw_save(idx(1:end-2));
    
    zero_yaw=[];
    yaw_index=1;
    for i=1:length(idx)-1
        j=idx(i)+ceil((idx(i+1)-idx(i))/3);
        while j < idx(i+1)
            if abs(yaw_save(j)-yaw_save(j-1))<50
                zero_yaw(yaw_index) = yaw_save(j)-yaw_save(j-1);
                yaw_index=yaw_index+1;
            end
            j=j+ceil((idx(i+1)-idx(i))/6);
        end
    end
    
    loss = sum( sqrt((PositionX(idx)-markpoint(:,2)).^2 +(PositionY(idx)-markpoint(:,3)).^2))+sum(delta_markpoint(2:end-1)-delta_yaw(2:end-1))+sum(zero_yaw);
end