function [PositionX,PositionY] = recalcu_posi(x)
    load('cur_data.mat')
    step_sz=[0.6933 -0.01902 0.005733];
    yaw=[1 0];
    bias=[0 0 0 0 0 0 0 0 0];
    if length(x)==7
        step_sz = x(1:3);
    %     step_sz=[0.6933 -0.01902 0.005733];
    %     yaw_w=x(4:5);
        Ki= x(4);
        Kp=x(5);
        yaw_w=x(6:7);
    elseif length(x)==14
        step_sz = x(1:3);
        yaw_w=x(4:5);
        bias = x(6:14);
    end
    rad2deg=180/pi;
    deg2rad=pi/180;
    tt(1)=0;                                      %算法单次仿真时间测量
    Fs=100;
    SamplePeriod=1/Fs;
    trial=trial_all;
    Length= size(trial,1);   
    %% 计算初始角
    st=0;
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
 
    %% 计算航向角变化
    for t = 2:Length
        tic;
    %     q(t,:) = madgwickAHRS(Gyroscope(t,:), Accelerometer(t,:), Magnetometer(t,:),q(t-1, :),Beta,SamplePeriod);	%梯度下降法  
       [q(t,:),eInt(t,:)] = MahonyAHRSupdate( [gx(t) gy(t) gz(t)], [ax(t) ay(t) az(t)], [mx(t) my(t) mz(t)],q(t-1, :),SamplePeriod,Ki,Kp,eInt(t-1,:));%Mahony法
        tt(t)=toc;%计算算法运行时间

        T=[ 1 - 2 * (q(t,4) *q(t,4) + q(t,3) * q(t,3)) 2 * (q(t,2) * q(t,3) +q(t,1) * q(t,4)) 2 * (q(t,2) * q(t,4)-q(t,1) * q(t,3));
            2 * (q(t,2) * q(t,3)-q(t,1) * q(t,4)) 1 - 2 * (q(t,4) *q(t,4) + q(t,2) * q(t,2)) 2 * (q(t,3) * q(t,4)+q(t,1) * q(t,2));
            2 * (q(t,2) * q(t,4) +q(t,1) * q(t,3)) 2 * (q(t,3) * q(t,4)-q(t,1) * q(t,2)) 1 - 2 * (q(t,2) *q(t,2) + q(t,3) * q(t,3))];%cnb
        Roll(t)  = atan2(T(2,3),T(3,3))*rad2deg;
        Pitch(t) = asin(-T(1,3))*rad2deg;
        Yaw(t)   = atan2(T(1,2),T(1,1))*rad2deg-8.3;  
    end
     addpath('quaternion_library');      % include quaternion library
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
% 
    yaw=euler(:,3);
    yaw=yaw_w(1)*yaw+yaw_w(2);
%     yaw=yaw_w(1)*(yaw.^2)+yaw_w(2)*yaw+yaw_w(3);
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
%     distance = 0;
    for k = 1:StepCount-1
        pos_start = PeakLocation(k);
        pos_end = PeakLocation(k+1);
       ftime(k)= trial(pos_start,2);
        % orientation (yaw)
        YawSin = mean(yaw(pos_start:pos_end,2));
        YawCos = mean(yaw(pos_start:pos_end,3));
    %    orientation(k,1:4)=[mean(yaw(pos_start:pos_end,2)),mean(yaw(pos_start:pos_end,3)),mean(euler(pos_start:pos_end,3)),mean(euler(pos_start:pos_end,3))];

        % step length estimation
        % SL = 0.2844 + 0.2231*frequency + 0.0426*AV
        StepFreq = 500/PkValue(k+1);
        StepAV = var(acc(pos_start:pos_end));
         %StepLength = 0.2844 + 0.2231*StepFreq + 0.0426*StepAV;%开源工程原有参数
         StepLength = step_sz(1) +step_sz(2)*StepFreq + step_sz(3)*StepAV; %消防模块拟合出来的步长参数
        % StepLength = 4.484 -4.183*StepFreq + 0.1115*StepAV; %消防模块拟合出来的步长参数
         % StepLength = 2.799 -2.817*StepFreq + 0.2988*StepAV; %消防模块拟合出来的步长参数
    %     StepLength=0.65;
%         distance = distance + StepLength;
    %    StepLengthArr(k)=StepLength;
        PositionY(k+1) = PositionY(k) + StepLength * sin(YawSin*deg2rad);
        PositionX(k+1) = PositionX(k) - StepLength * cos(YawCos*deg2rad);
    end
%     end_pose=[Pitch(end) Roll(end) Yaw(end) eInt(end,:)];
    figure

    scatter(PositionX(:),PositionY(:),'b'); grid on; % north-up
    hold on;

    scatter(markpoint(:,2),markpoint(:,3),'r'); 
    plot(markpoint(:,2),markpoint(:,3),'b');
    
    idx=[];
    for i=1:length(markpoint)
        deltat = abs(ftime-markpoint(i,1)*1000);
        idx(i)=find(deltat==min(deltat));
    end
    scatter(PositionX(idx),PositionY(idx),'g'); 
end