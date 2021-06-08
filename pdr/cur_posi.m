function cur_posi(x)
    align_path='/home/mxc/wonderland/ipin_data/01-Training';
    align_folder = dir(align_path);
    trial = [];
    true_posi=[];
    trial_idx=1;
    
    addpath('utils'); 
    loss=0;
    for i=3:length(align_folder)
        data_p=strcat(align_folder(i).folder,'/',align_folder(i).name);
        align_subfolder = dir(data_p);
        for j=3:2:length(align_subfolder)
            data_s_p = strcat(align_subfolder(j).folder,'/',align_subfolder(j).name);
            file_p=strcat(data_s_p,'/ACCEGYROMAGNAHRS.csv');
            posi_p=strcat(data_s_p,'/POSI.csv');
            trial=dlmread(file_p,',',4,1);
            markpoint = dlmread(posi_p,',',1,2);
            
            new_position=GPSToXY(markpoint(:,[3 4]));
            new_position=new_position-new_position(1,:);
            markpoint(:,2:3) = new_position;
            startTime=zhidao_nearest(trial(:,1),markpoint(1,1));
            startIndex=find(trial(:,1)==startTime);

            %  stopTime=zhidao_nearest(trial(:,2),markpoint(8,1)*1000);
            %  stopIndex=find(trial(:,2)==stopTime);
            %  stopIndex=0;
            trial=trial(startIndex:end,:);
            addpath('SDK'); 
            Accelerometer=trial(:,3:5);
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
            time = trial(:,1) ; % each sample's collect time

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

            Accelerometer=[LPFAx,LPFAy,LPFAz];
            ax=Accelerometer(:,1); %%加速度计数据
            ay=Accelerometer(:,2); 
            az=Accelerometer(:,3); 

            rad2deg=180/pi;
            deg2rad=pi/180;
            time(1)=0;
            
            % figure
            temp=trial(:,23);
            for i=1:length(temp)
                if temp(i)<0
                    temp(i)=temp(i)+360;
                end
            end
            yaw=temp;
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

            %% position update
            [PkValue, PeakLocation] = stateStepDetection(ax, ay, az);
            % PkValue = [zeros(length(PkValue),1) PkValue];
            StepCount = length(PkValue);
            PositionX = zeros(StepCount, 1);
            PositionY = zeros(StepCount, 1);

%             figure
            PositionX(1)=0;
            PositionY(1) = 0;

            distance = 0;
            StepLengthArr=[];
%             orientation=[];
            yaw_get=[];
            ftime=[];
            for k = 1:StepCount-1
                pos_start = PeakLocation(k);
                pos_end = PeakLocation(k+1);
                % orientation (yaw)
                YawSin = mean(yaw(pos_start:pos_end,2));
                YawCos = mean(yaw(pos_start:pos_end,3));
                ftime(k)=trial(pos_start,1);
                % step length estimation
                % SL = 0.2844 + 0.2231*frequency + 0.0426*AV
                StepFreq = 1000/(PkValue(k)*2);
                StepAV = var(acc(pos_start:pos_end));
                 StepLength = x(1) + x(2)*StepFreq + x(3)*StepAV; %消防模块拟合出来的步长参数
                distance = distance + StepLength;
                StepLengthArr(k)=StepLength;
                PositionX(k+1) = PositionX(k) + StepLength * sin(YawSin*deg2rad);
                PositionY(k+1) = PositionY(k) - StepLength * cos(YawCos*deg2rad);
%                 scatter(-PositionX(k+1),PositionY(k+1),'b'); grid on; % north-up
%                 hold on;
%                   pause(0.01)
            end
            PositionX=-PositionX;
            idx=[];
            for i=1:length(markpoint(:,1))
                deltat = abs(ftime-markpoint(i,1));
                idx(i)=find(deltat==min(deltat));
            end
%             loss = loss + sum( sqrt((PositionX(idx)-markpoint(:,2)).^2 +(PositionY(idx)-markpoint(:,3)).^2));
            figure
            plot(markpoint(:,2),markpoint(:,3),'r');hold on;
            scatter(PositionX,PositionY,'b');
        end
    end
end