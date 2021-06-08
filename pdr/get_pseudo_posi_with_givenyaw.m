function get_pseudo_posi_with_givenyaw(posi_p,file_p,save_p,file_n,x) 
    step_sz=[0.5933 -0.01902 0.005733];
%     step_sz=[  0.5533    0.0187   -0.0530];
    yaw_w=[1 0];
    bias=[0 0 0 0 0 0 0 0 0];
    Kp=0.002;
    Ki=0;
    if length(x)==7
        step_sz = x(1:3);
        Ki= x(4);
        Kp=x(5);
        yaw_w=x(6:7);
    elseif length(x)==12
        step_sz = x(1:3);
        bias = x(4:12);  
    elseif length(x)==14
        step_sz = x(1:3);
        yaw_w=x(4:5);
        bias = x(6:14);      
    end
    addpath('utils');
%     markpoint = dlmread(posi_p,',',1,2);
%     new_position=GPSToXY(markpoint(:,[3 4]));
%     new_position=new_position-new_position(1,:);
%     markpoint(:,2:3) = new_position;
    markpoint=dlmread(posi_p,',',2,2);
    position=markpoint(:,[3 4]);
    position(:,3)=0;
    orgxyz=position(1,:);
    llh=position;
    enu=llh2enu(orgxyz,llh);
    markpoint(:,2:3)=enu(:,1:2);
%     figure
%     plot(markpoint(:,2),markpoint(:,3))
    %% latlon to ~
    %%
%     GPS_pos=markpoint(:,2:3);
%     new_position=GPSToXY(markpoint(:,[2 3]));
%     new_position=new_position-new_position(1,:);
%     % plot(new_position(:,1),new_position(:,2));
%     % figure
% 
%     markpoint(:,[2 3])=markpoint(:,[3 2]);
%     longitudefactor = 85000;
%     latitudefactor = 110000;
%     X=zeros(length(markpoint),1);
%     Y=zeros(length(markpoint),1);
%     for i=2:length(markpoint)
%         X(i)= (markpoint(i,2)-markpoint(1,2))*longitudefactor;
%         Y(i)= (markpoint(i,3)-markpoint(1,3))*latitudefactor;
%     end
%     % plot(X,Y)
%     markpoint(:,2)=X;
%     markpoint(:,3)=Y;
        % file='ipin2018/Logfiles/processed/Training/T02_02.txt'
    trial=dlmread(file_p,',',4,1);
    startTime=zhidao_nearest(trial(:,1),markpoint(1,1));
    startIndex=find(trial(:,1)==startTime);
    stopTime=zhidao_nearest(trial(:,1),markpoint(end,1));
    stopIndex=find(trial(:,1)==stopTime);
    trial=trial(startIndex:stopIndex,:);
    addpath('SDK'); 
%     Gyroscope=trial(:,9:11);
    Accelerometer=trial(:,3:5);
%     Magnetometer=trial(:,15:17);
    % figure


    %% 低通滤波 室内alpha=0.975   室外alpha=0.67
    accc=zeros(length(Accelerometer),3);
%     mag=zeros(length(Magnetometer),3);
%     gyro=zeros(length(Gyroscope),3);
    alpha=0.975;

    for i=2:length(Accelerometer)
        accc(i,:) = alpha * accc(i-1,:) + (1 - alpha)* Accelerometer(i,:);
%          mag(i,:) = alpha * mag(i-1,:) + (1 - alpha)* Magnetometer(i,:);
%          gyro(i,:) = alpha * gyro(i-1,:) + (1 - alpha)* Gyroscope(i,:);
    end

    %%

%     gx=Gyroscope(:,1)+bias(1); %%陀螺仪数据
%     gy=Gyroscope(:,2)+bias(2); 
%     gz=Gyroscope(:,3)+bias(3); 
    ax=Accelerometer(:,1)+bias(4); %%加速度计数据
    ay=Accelerometer(:,2)+bias(5); 
    az=Accelerometer(:,3)+bias(6); 
%     mx=Magnetometer(:,1)+bias(7); %%磁强计数据
%     my=Magnetometer(:,2)+bias(8); 
%     mz=Magnetometer(:,3)+bias(9); 
    time = trial(:,1) ; % each sample's collect time
    
    %%  
    % Magnititude of acceleration data from experiment data
%     mag = sqrt(ax.^2+ay.^2+az.^2);
    % Non-gravity of acceleration
%     magNoG = mag - mean(mag);
    % Magnititude of gyroscope data from experiment data
%     gyro = sqrt(gx.^2+gy.^2+gz.^2);
    % Samping frequency of experiment
%     Fs = length(time) / (time(length(time)) - time(1))*1000;
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
%     LPFMx = filtfilt(b, a, mx);
%     LPFMy = filtfilt(b, a, my);
%     LPFMz = filtfilt(b, a, mz);
%     LPFGx = filtfilt(b, a, gx);
%     LPFGy = filtfilt(b, a, gy);
%     LPFGz = filtfilt(b, a, gz);

%     LPFGyro = filtfilt(b,a,gyro); % LPF output raw gyroscope data

    Accelerometer=[LPFAx,LPFAy,LPFAz];
%     Gyroscope=[LPFGx,LPFGy,LPFGz];
%     Magnetometer=[LPFMx,LPFMy,LPFMz];
%     gx=Gyroscope(:,1); %%陀螺仪数据
%     gy=Gyroscope(:,2); 
%     gz=Gyroscope(:,3); 
    ax=Accelerometer(:,1); %%加速度计数据
    ay=Accelerometer(:,2); 
    az=Accelerometer(:,3); 
%     mx=Magnetometer(:,1); %%磁强计数据
%     my=Magnetometer(:,2); 
%     mz=Magnetometer(:,3);

    rad2deg=180/pi;
    deg2rad=pi/180;
    time(1)=0;
    quaternion = zeros(length(time), 4);
    % figure
    acc = sqrt(Accelerometer(:,1).^2 + Accelerometer(:,2).^2 + Accelerometer(:,3).^2);
    acc = acc-mean(acc);
    %% BANDPASS FILTER
    fs=200;
    f1=0.75;               % cuttoff low frequency to get rid of baseline wander
    f2=2.75;                 % cuttoff frequency to discard high frequency noise
    Wn=[f1 f2]/(fs/2);    % cutt off frequency based on fs
    N = 4;                % order of 3 less processing

    [a,b] = butter(N,Wn); % bandpass filtering
    bandPass = filtfilt(a,b,acc);
%     figure
%     plot(acc)
    SamplePeriod=1/Fs;                            %数据采样时间
    
    Length= size(trial,1);   
    %% 计算初始角
%     index=2;
%     Pitch0=asin(ax(index)/sqrt(ax(index)*ax(index)+ay(index)*ay(index)+az(index)*az(index)));%初始角计算
%     Roll0=atan2(-ay(index),-az(index));
%     Yaw0=atan2(-my(index)*cos(Roll0)+mz(index)*sin(Roll0),mx(index)*cos(Pitch0)+my(index)*sin(Pitch0)*sin(Roll0)+mz(index)*sin(Pitch0)*cos(Roll0))-8.3*pi/180;
% 
%     Pitch(1)= Pitch0*rad2deg; 
%     Roll(1) =Roll0*rad2deg;
%     Yaw(1) = Yaw0*rad2deg;
% 
%     eInt(1,:)=[0 0 0];                            %叉积法误差初值
%     
%     % Yaw0*rad2deg;
%     [q0,q1,q2,q3]=Quaternion_FromEulerAngle(Roll0,Pitch0,Yaw0);%初始四元数计算
%     q(1,:)=[q0,q1,q2,q3];
 

     addpath('quaternion_library');      % include quaternion library
    %% Process sensor data through algorithm

    %  AHRS = MadgwickAHRS('SamplePeriod', 1/100, 'Beta', 0.1);
%     AHRS = MahonyAHRS('SamplePeriod', SamplePeriod, 'Kp', Kp,'Ki',Ki);

%     quaternion = zeros(length(trial), 4);
%     quaternion(1,:)=q(1,:);
%     for t = 2:length(trial)
%         AHRS.Update(Gyroscope(t,:), Accelerometer(t,:), Magnetometer(t,:),quaternion(t-1, :));	% gyroscope units must be radians
%     %     AHRS.UpdateIMU(Gyroscope(t,:), Accelerometer(t,:));	% gyroscope units must be radians
%         quaternion(t, :) = AHRS.Quaternion;
% 
%     end
%     
%     euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.
%     for i=1:length(euler)
%         if euler(i,3)<0
%             euler(i,3)=euler(i,3)+360;
%         end
%     end
% 
%     yaw=euler(:,3);
    temp=trial(:,23);
    for i=1:length(temp)
        if temp(i)<0
            temp(i)=temp(i)+360;
        end
    end
    yaw=temp;
    yaw=yaw_w(1)*yaw+yaw_w(2);
    temp_yaw=yaw;
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
    step_idx=[];
     ptime=[];
    pseudo_pos=[];
    pseudo_pos(1,1)=markpoint(1,2);
    pseudo_pos(1,2)=markpoint(1,3);
    cnt=0;
    pseudo_idx=1;
    global_idx=1;
    ptime(pseudo_idx)=markpoint(1,1);
%     for i=1:round(PeakLocation(1)/100)
%         pstep_idx(pseudo_idx,1:2)=[global_idx global_idx+100];
%         ptime(pseudo_idx)=trial(global_idx,1);
%         pseudo_pos(pseudo_idx+1,:) = pseudo_pos(pseudo_idx,:);
%         pseudo_idx=pseudo_idx+1;
%         global_idx = global_idx+100;
%     end
    yaw_save=[];
    step_save=[];
    for k = 1:StepCount-1
        pos_start = PeakLocation(k);
        pos_end = PeakLocation(k+1);
        step_idx(k,1:2)=[pos_start pos_end];
        YawSin = mean(yaw(pos_start:pos_end,2));
        YawCos = mean(yaw(pos_start:pos_end,3));
        ftime(k)=trial(pos_start,1);
        StepFreq = 500/(PkValue(k+1));
        StepAV = var(acc(pos_start:pos_end));
        StepLength = step_sz(1) + step_sz(2)*StepFreq + step_sz(3)*StepAV; 
        PositionX(k+1) = PositionX(k) + StepLength * sin(YawSin*deg2rad);
        PositionY(k+1) = PositionY(k) - StepLength * cos(YawCos*deg2rad);
        StepLength=StepLength/round((pos_end-pos_start)/100);
        if pos_end-pos_start>1000
            for i= round((pos_start)/100)*100:100: round((pos_end)/100)*100-100
                pstep_idx(pseudo_idx,1:2)=[i i+100];
                ptime(pseudo_idx)=trial(i,1);
                yaw_save=[yaw_save;mean(temp_yaw(pos_start:pos_end))];
                step_save=[step_save;step_save(length(step_save(:,1)),:)];
                pseudo_pos(pseudo_idx+1,2) = pseudo_pos(pseudo_idx,2) ;
                pseudo_pos(pseudo_idx+1,1) = pseudo_pos(pseudo_idx,1) ;
                pseudo_idx=pseudo_idx+1;
            end
        else
            for i= round((pos_start)/100)*100:100: round((pos_end)/100)*100-100
                if i==0
                    i=1;
                end
                yaw_save=[yaw_save;mean(temp_yaw(pos_start:pos_end))];
                pstep_idx(pseudo_idx,1:2)=[i i+100];
                ptime(pseudo_idx)=trial(i,1);
                step_save=[step_save;(StepLength * sin(YawSin*deg2rad)) (-StepLength * cos(YawCos*deg2rad))];
                pseudo_pos(pseudo_idx+1,1) = pseudo_pos(pseudo_idx,1) + StepLength * sin(YawSin*deg2rad);
                pseudo_pos(pseudo_idx+1,2) = pseudo_pos(pseudo_idx,2) - StepLength * cos(YawCos*deg2rad);
                pseudo_idx=pseudo_idx+1;
            end
        end
    end
     global_idx = pstep_idx(pseudo_idx-1,2);
    while global_idx < length(trial(:,1))
        if length(trial(:,1))-global_idx < 100
            break;
        end
        pos_start = global_idx;
        pos_end = min(global_idx+100,length(trial(:,1)));
        pstep_idx(pseudo_idx,1:2)=[pos_start pos_end];
        YawSin = mean(yaw(pos_start:pos_end,2));
        YawCos = mean(yaw(pos_start:pos_end,3));
        yaw_save=[yaw_save;mean(temp_yaw(pos_start:pos_end))];
        ptime(pseudo_idx)=trial(pos_start,1);
        StepFreq = 500/(PkValue(end));
        StepAV = var(acc(pos_start:pos_end));
        StepLength = step_sz(1) + step_sz(2)*StepFreq + step_sz(3)*StepAV; 
        step_save=[step_save;(StepLength * sin(YawSin*deg2rad)) (-StepLength * cos(YawCos*deg2rad))];
        pseudo_pos(pseudo_idx+1,1) = pseudo_pos(pseudo_idx,1) + StepLength * sin(YawSin*deg2rad);
        pseudo_pos(pseudo_idx+1,2) = pseudo_pos(pseudo_idx,2) - StepLength * cos(YawCos*deg2rad);
        pseudo_idx=pseudo_idx+1;
        global_idx = global_idx+100;
        
    end
    idx=[];
    for i=1:length(markpoint(:,1))
        deltat = abs(ftime-markpoint(i,1));
        temp_idx=find(deltat==min(deltat));
        idx(i)=temp_idx(1);
    end
    
    pidx=[];
    for i=1:length(idx)
        deltat = abs(ptime-markpoint(i,1));
        pidx(i)=find(deltat==min(deltat));
    end
%     addition_step=addition_markpoint(ptime,pidx,pstep_idx,markpoint,trial,yaw,pseudo_pos(:,1),pseudo_pos(:,2),acc,step_sz);
%     addition_step(:,1)=-addition_step(:,1);
%     pseudo_pos(:,1)=-pseudo_pos(:,1);
    
    figure
    plot(markpoint(:,2),markpoint(:,3),'b');
    hold on;
    scatter(-pseudo_pos(:,1),-pseudo_pos(:,2),'r');
    c=['r' 'g' 'b' 'm' 'c'];
    k=1;
    res=[];
    while k<length(markpoint(:,1))
        st = pidx(k);
        if k==length(markpoint(:,1))-1
            ed = pidx(k+1);
        else
            ed= pidx(k+1)-1;
        end
        [x_trans,y_trans]=rotation_segment(pseudo_pos(st:ed,1),pseudo_pos(st:ed,2),markpoint(k:k+1,:));%,addition_step(k:k+1,:));
%         scatter(PositionX(idx(k):idx(ed)),PositionY(idx(k):idx(ed)),'k');
%         scatter(x_trans,y_trans,c(mod(k,5)+1));
        res=[res; x_trans y_trans];
        k=k+1;
    end
%     figure
%     plot(-markpoint(:,2),-markpoint(:,3),'b')
%     hold on
    scatter(res(:,1),res(:,2),'b');
%     
    res(:,3)=0;
    res=enu2llh(res,orgxyz');
    res = [ptime' res(:,1:2) yaw_save step_save];
    
    dlmwrite(strcat(save_p,file_n,'.csv'),res,'precision','%.15f');
end


function [positionX,positionY]=rotation_segment(x,y,mkp)%,addition_step)
%     if flag_step=='last' 
%         x = [ addition_step(1,1);x; addition_step(2,1)];
%         y = [ addition_step(1,2);y; addition_step(2,2)];
%     elseif flag_step=='next'
%         x = [addition_step(1,1);x; addition_step(2,1)];
%         y = [addition_step(1,2);y; addition_step(2,2)];        
%     end
    step_count = length(x);
%     if step_count<2
%         positionX=(mkp(end,2)+mkp(1,2))/2;
%         positionY=(mkp(end,3)+mkp(1,3))/2;
%     else
        u = [mkp(end,2)-mkp(1,2) mkp(end,3)-mkp(1,3) 0];
        v = [x(step_count)-x(1) y(step_count)-y(1) 0];
        if (norm(u)*norm(v))==0
            theta=0;
            rho=[0 0 0];
        else
            theta = acos(u*v'/(norm(u)*norm(v)));
            rho = asin(cross(v,u)/(norm(u)*norm(v)));
        end
        if rho(3)<0
            theta = -theta;
        end
        position = [x y ones(step_count,1)];
        t = [1 0 0;0 1 0;-x(1) -y(1) 1];
        r = [cos(theta) sin(theta) 0;-sin(theta) cos(theta) 0;0 0 1];
        pos_r = position*t*r;
        t_m=[1 0 0;0 1 0;mkp(1,2) mkp(1,3) 1];
%         sx = (mkp(end,2)-mkp(1,2))/(pos_r(step_count,1)-pos_r(1,1))*(step_count-1)/step_count;
%         sy = (mkp(end,3)-mkp(1,3))/(pos_r(step_count,2)-pos_r(1,2))*(step_count-1)/step_count;
        if norm(v)==0
            sx = 1;
        else
            sx = norm(u)/norm(v);
        end
        sy=sx;
        s = [sx 0 0;0 sy 0;0 0 1];
        position=pos_r*s*t_m;
        
        positionX = position(:,1);
        positionY = position(:,2); 
%         positionX = position(2:end-1,1);
%         positionY = position(2:end-1,2); 
%     end
end

function addition_step=addition_markpoint(ftime,idx,step_idx,markpoint,trial,yaw,PositionX,PositionY,acc,step_sz)
    deg2rad = 180/pi;
    for i=1:length(markpoint)
        if ftime(idx(i))>markpoint(i,1)
            flag_step(i,1:4)='next';
            pos_end=step_idx(idx(i),1);
            pos_start=find(trial(:,1)==zhidao_nearest(trial(:,1),markpoint(i,1)));
            YawSin = mean(yaw(pos_start:pos_end,2));
            YawCos = mean(yaw(pos_start:pos_end,3));
            StepFreq = 500/(pos_end-pos_start);
            StepAV = var(acc(pos_start:pos_end));
            StepLength = step_sz(1) + step_sz(2)*StepFreq + step_sz(3)*StepAV; 
            addition_step(i,1) = PositionX(idx(i)) - StepLength * sin(YawSin*deg2rad);
            addition_step(i,2) = PositionY(idx(i)) + StepLength * cos(YawCos*deg2rad);
        
        elseif ftime(idx(i))<markpoint(i,1)
            flag_step(i,1:4)='last';
            pos_start=step_idx(idx(i),1);
            tempidx=find(trial(:,1)==zhidao_nearest(trial(:,1),markpoint(i,1)));
            pos_end=tempidx(1);
            flag=1;
            if pos_start>pos_end
                flag=-1;
                te=pos_start;
                pos_start=pos_end;
                pos_end=te;
            end            
            YawSin = mean(yaw(pos_start:pos_end,2));
            YawCos = mean(yaw(pos_start:pos_end,3));
            StepFreq = 500/(pos_end-pos_start);
            StepAV = var(acc(pos_start:pos_end));
            StepLength = step_sz(1) + step_sz(2)*StepFreq + step_sz(3)*StepAV; 
            addition_step(i,1) = PositionX(idx(i)) + flag*StepLength * sin(YawSin*deg2rad);
            addition_step(i,2) = PositionY(idx(i)) - flag*StepLength * cos(YawCos*deg2rad);
        else
            flag_step(i,1:4)='curt';
            addition_step(i,1) = PositionX(idx(i));
            addition_step(i,2) = PositionY(idx(i));
        end
    end
end
