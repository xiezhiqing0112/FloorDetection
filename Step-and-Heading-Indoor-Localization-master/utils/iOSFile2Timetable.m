function data  = iOSFile2Timetable(file_path)
% Script for importing data from the following text file:
%
%    filename: /home/default/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/datasets/okt 6 house session/lopen3/2020-10-06_15_02_12_my_iOS_device.csv
%
% Auto-generated by MATLAB on 08-Oct-2020 19:18:45

%% Setup the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 79);

% Specify range and delimiter
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["Time", "loggingSample", "identifierForVendor", ...
                      "deviceID", "locationTimestamp_since1970", "locationLatitude", ...
                      "locationLongitude", "locationAltitude", "locationSpeed", ...
                      "locationCourse", "locationVerticalAccuracy", "locationHorizontalAccuracy", ...
                      "locationFloor", "locationHeadingTimestamp_since1970", "locationHeadingX",...
                      "locationHeadingY", "locationHeadingZ", "locationTrueHeading", ...
                      "locationMagneticHeading", "locationHeadingAccuracy",...
                      "accelerometerTimestamp_sinceReboot", "accelerometerAccelerationX",...
                      "accelerometerAccelerationY", "accelerometerAccelerationZ", ...
                      "gyroTimestamp_sinceReboot", "gyroRotationX", "gyroRotationY",...
                      "gyroRotationZ", "magnetometerTimestamp_sinceReboot", "magnetometerX",...
                      "magnetometerY", "magnetometerZ", "motionTimestamp_sinceReboot",...
                      "motionYaw", "motionRoll", "motionPitch", "motionRotationRateX",...
                      "motionRotationRateY", "motionRotationRateZ", "motionUserAccelerationX",...
                      "motionUserAccelerationY", "motionUserAccelerationZ", ...
                      "motionAttitudeReferenceFrame", "motionQuaternionX", "motionQuaternionY",...
                      "motionQuaternionZ", "motionQuaternionW", "motionGravityX", ...
                      "motionGravityY", "motionGravityZ", "motionMagneticFieldX",...
                      "motionMagneticFieldY", "motionMagneticFieldZ", ...
                      "motionMagneticFieldCalibrationAccuracy", ...
                      "activityTimestamp_sinceReboot", "activity", ...
                      "activityActivityConfidence", "activityActivityStartDate",...
                      "pedometerStartDate", "pedometerNumberofSteps", "pedometerAverageActivePace",...
                      "pedometerCurrentPace", "pedometerCurrentCadence", ...
                      "pedometerDistance", "pedometerFloorAscended", "pedometerFloorDescended",...
                      "pedometerEndDate", "altimeterTimestamp_sinceReboot", "altimeterReset", ...
                      "altimeterRelativeAltitude", "altimeterPressure", "IP_en0", "IP_pdp_ip0", ...
                      "deviceOrientation", "batteryState", "batteryLevel", ...
                      "avAudioRecorderPeakPower", "avAudioRecorderAveragePower", "label"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
data = readtable(file_path, opts);

data.Time = datetime(data.Time,'ConvertFrom','posixtime');
data.Time.TimeZone = 'UTC';
data.Time.Format = 'yyyy-MM-dd HH:mm:ss.SSS';

data.Date = data.Time ;
data.Time = data.Time - data.Time(1);
data.Time = seconds(data.Time);
data.Time = seconds(data.Time);
data = table2timetable(data);

data.door_open = zeros(height(data),1);
%% Clear temporary variables
clear opts