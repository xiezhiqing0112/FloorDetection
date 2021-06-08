function loss = segment_pos(x)%st,ed,markpoint_index)
    step_sz = x(1:3);
% %     step_sz=[0.6933 -0.01902 0.005733];
    Ki = x(4);
    Kp = x(5);
    load 'all_data.mat'
    trial=trial_all(st:ed);
%     step_sz=[ 1.456309376961421  0.074417104729315   0.290514118994777];
%     Ki= -0.181743536483214;
%     Kp = -0.003050424864017;
    rad2deg=180/pi;
    deg2rad=pi/180;
    tt(1)=0;                                      %算法单次仿真时间测量
    Fs=100;
    SamplePeriod=1/Fs;                            %数据采样时间
    
%     Beta=0.009;                                    %梯度下降法梯度步长设置
                                    %叉积法比例积分系数设置
   
%     Length= size(trial,1x);   
    %% 计算初始角
    index=st+2;
    Pitch0=asin(ax(index)/sqrt(ax(index)*ax(index)+ay(index)*ay(index)+az(index)*az(index)));%初始角计算
    Roll0=atan2(-ay(index),-az(index));
    Yaw0=atan2(-my(index)*cos(Roll0)+mz(index)*sin(Roll0),mx(index)*cos(Pitch0)+my(index)*sin(Pitch0)*sin(Roll0)+mz(index)*sin(Pitch0)*cos(Roll0))-8.3*pi/180;

    Pitch(1)= Pitch0*rad2deg; 
    Roll(1) =Roll0*rad2deg;
    Yaw(1) = Yaw0*rad2deg;

     eInt(1,:)=[0 0 0];                            %叉积法误差初值
    
    % Yaw0*rad2deg;
    [q0,q1,q2,q3]=Quaternion_FromEulerAngle(Roll0,Pitch0,Yaw0);%初始四元数计算
%     [q0,q1,q2,q3]=Quaternion_FromEulerAngle(0,0,90*deg2rad);%初始四元数计算
    q(1,:)=[q0,q1,q2,q3];
    
     addpath('quaternion_library'); 
    %% Process sensor data through algorithm

    %  AHRS = MadgwickAHRS('SamplePeriod', 1/100, 'Beta', 0.1);
    AHRS = MahonyAHRS('SamplePeriod', SamplePeriod, 'Kp', Kp,'Ki',Ki);

    quaternion = zeros(length(trial), 4);
    quaternion(1,:)=q(1,:);
    for t = 2:length(trial)
        AHRS.Update(Gyroscope(st+t,:), Accelerometer(st+t,:), Magnetometer(st+t,:),quaternion(t-1, :));	% gyroscope units must be radians
    %     AHRS.UpdateIMU(Gyroscope(t,:), Accelerometer(t,:));	% gyroscope units must be radians
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
    
%     flatMagnetic = quaternRotate(Magnetometer(st:ed,:), quaternConj(quaternion));

    [PkValue, PeakLocation] = stateStepDetection(ax(st:ed),ay(st:ed),az(st:ed));
    StepCount=length(PkValue);
    %% position update
    PositionX = zeros(StepCount, 1);
    PositionY = zeros(StepCount, 1);
    PositionX(1)=markpoint_rot(markpoint_index,1);
    PositionY(1)=markpoint_rot(markpoint_index,2);
%     distance = 0;
    for k = 1:StepCount-1
        pos_start = PeakLocation(k);
        pos_end = PeakLocation(k+1);
    %    ftime(k)=trial(pos_start,2);
        % orientation (yaw)
        YawSin = mean(yaw(pos_start:pos_end,2));
        YawCos = mean(yaw(pos_start:pos_end,3));
    %    orientation(k,1:4)=[mean(yaw(pos_start:pos_end,2)),mean(yaw(pos_start:pos_end,3)),mean(euler(pos_start:pos_end,3)),mean(euler(pos_start:pos_end,3))];

        % step length estimation
        % SL = 0.2844 + 0.2231*frequency + 0.0426*AV
        StepFreq = 1000/PkValue(k+1);
        StepAV = var(acc(st+pos_start:st+pos_end));
        StepLength = step_sz(1) - step_sz(2)*StepFreq + step_sz(3)*StepAV; 
        PositionX(k+1) = PositionX(k) + StepLength * sin(YawSin*deg2rad);
        PositionY(k+1) = PositionY(k) - StepLength * cos(YawCos*deg2rad);
    end
%     Position=[PositionX PositionY];
    %% 计算误差
    
    loss =sqrt((PositionX(end)-markpoint_rot(markpoint_index+1,1))^2+(PositionY(end)-markpoint_rot(markpoint_index+1,2))^2);
    
%     step_matrix=zeros(StepCount-1,3);
%     YawSindeg2rad_matrix=zeros(StepCount-1,1);
%     YawCosdeg2rad_matrix=zeros(StepCount-1,1);
%     for k = 1:StepCount-1
%         pos_start = PeakLocation(k);
%         pos_end = PeakLocation(k+1);
%     %    ftime(k)=trial(pos_start,2);
%         % orientation (yaw)
%         YawSin = mean(yaw(pos_start:pos_end,2));
%         YawCos = mean(yaw(pos_start:pos_end,3));
%     %    orientation(k,1:4)=[mean(yaw(pos_start:pos_end,2)),mean(yaw(pos_start:pos_end,3)),mean(euler(pos_start:pos_end,3)),mean(euler(pos_start:pos_end,3))];
% 
%         % step length estimation
%         % SL = 0.2844 + 0.2231*frequency + 0.0426*AV
%         StepFreq = 1000/PkValue(k+1,2);
%         StepAV = var(acc(pos_start:pos_end));
%         step_matrix(k,:)=[1 -StepFreq StepAV];   
%         YawSindeg2rad_matrix(k)=(StepCount-k)* sin(YawSin*deg2rad);
%         YawCosdeg2rad_matrix(k)=(StepCount-k)* cos(YawCos*deg2rad);
%     end
%     loss = sqrt((sum(YawSindeg2rad_matrix'*(step_sz*step_matrix')')+PositionX(1)-markpoint_rot(markpoint_index+1,1))^2+(PositionY(1)-sum(YawCosdeg2rad_matrix'*(step_sz*step_matrix')')-markpoint_rot(markpoint_index+1,2))^2);
end