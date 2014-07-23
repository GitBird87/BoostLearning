%% -------------------------------------------------------------------------
clear;close all;clc;
addpath('quaternion_library');
addpath('ximu_matlab_library');
addpath('tools');

%Select dataset (comment in/out)

filePath = 'LoggedData/straightLine';
startTime = 6; stopTime = 26;

% filePath = 'LoggedData/stairsAndCorridor';
% startTime = 5; stopTime = 53;

% filePath = 'LoggedData/spiralStairs';
% startTime = 4; stopTime = 47;

%% -------------------------------------------------------------------------
% Import data
samplePeriod = 1/256; initPeriod = 3;
xIMUdata = xIMUdataClass(filePath, 'InertialMagneticSampleRate', 1/samplePeriod);
time = xIMUdata.CalInertialAndMagneticData.Time;
gyrX = xIMUdata.CalInertialAndMagneticData.Gyroscope.X;
gyrY = xIMUdata.CalInertialAndMagneticData.Gyroscope.Y;
gyrZ = xIMUdata.CalInertialAndMagneticData.Gyroscope.Z;
accX = xIMUdata.CalInertialAndMagneticData.Accelerometer.X;
accY = xIMUdata.CalInertialAndMagneticData.Accelerometer.Y;
accZ = xIMUdata.CalInertialAndMagneticData.Accelerometer.Z;
clear('xIMUdata');

%���н���������
downRate = 1;
time = time(1:downRate:length(time), :);
gyrX = gyrX(1:downRate:length(gyrX), :);
gyrY = gyrY(1:downRate:length(gyrY), :);
gyrZ = gyrZ(1:downRate:length(gyrZ), :);
accX = accX(1:downRate:length(accX), :);
accY = accY(1:downRate:length(accY), :);
accZ = accZ(1:downRate:length(accZ), :);
samplePeriod = samplePeriod * downRate ;

%% -------------------------------------------------------------------------
% ���ô����֡�� ��ʼλ�ú���ֹλ��
indexSel = find(sign(time-startTime)+1, 1) : find(sign(time-stopTime)+1, 1);
time = time(indexSel);
gyrX = gyrX(indexSel, :); gyrY = gyrY(indexSel, :); gyrZ = gyrZ(indexSel, :);
accX = accX(indexSel, :); accY = accY(indexSel, :); accZ = accZ(indexSel, :);

%% -------------------------------------------------------------------------
% ��̬֡���ļ��
% Compute accelerometer magnitude
acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);

% HP filter accelerometer data
filtCutOff = 0.001;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
acc_magFilt = filtfilt(b, a, acc_mag);

% Compute absolute value
acc_magFilt = abs(acc_magFilt);

% LP filter accelerometer data
filtCutOff = 6;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
acc_magFilt = filtfilt(b, a, acc_magFilt);

% Threshold detection
stationary = acc_magFilt < 0.05;

%% -------------------------------------------------------------------------
% ����ԭʼ�����ݺ;�̬ʱ�̵�����

figure('Position', [9 39 900 600], 'Number', 'off', 'Name', 'Sensor Data');

ax(1) = subplot(2,1,1); 
hold on;
plot(time, gyrX, 'r'); plot(time, gyrY, 'g'); plot(time, gyrZ, 'b');
title('Gyroscope'); xlabel('Time (s)'); ylabel('Angular velocity (^\circ/s)');
legend('X', 'Y', 'Z');
hold off;

ax(2) = subplot(2,1,2);
hold on;
plot(time, accX, 'r'); plot(time, accY, 'g'); plot(time, accZ, 'b'); 
plot(time, acc_magFilt, ':k'); plot(time, stationary, 'k', 'LineWidth', 2);
title('Accelerometer'); xlabel('Time (s)'); ylabel('Acceleration (g)');
legend('X', 'Y', 'Z', 'Filtered', 'Stationary');
hold off;

linkaxes(ax,'x');

%% -------------------------------------------------------------------------
% ���нǶȵļ���

quat = zeros(length(time), 4);
AHRSalgorithm = AHRS('SamplePeriod', samplePeriod, 'Kp', 1, 'KpInit', 1);

% Ѱ��Ŀ���ʼ��״̬��ʱ��� Ȼ���ڴ�ʱ��η�������Ѱ�ҳ�ʼʱ�̵���Ԫ��
indexSel = 1 : find( sign( time-( time(1) + initPeriod ) ) + 1, 1);
for i = 1:2000
    AHRSalgorithm.UpdateIMU([0 0 0], [mean(accX(indexSel)) mean(accY(indexSel)) mean(accZ(indexSel))]);
end

% �����е�Ԫ�ؽ�����Ԫ������
for t = 1:length(time)
    % Ŀ�괦�ھ�̬ʱ�̲Ž��������µ�Լ��
    if(stationary(t))
        AHRSalgorithm.Kp = 0.5;
    else
        AHRSalgorithm.Kp = 0;
    end
    AHRSalgorithm.UpdateIMU(deg2rad([gyrX(t) gyrY(t) gyrZ(t)]), [accX(t) accY(t) accZ(t)]);
    quat(t,:) = AHRSalgorithm.Quaternion;
end

%% -------------------------------------------------------------------------
% ���м��ٶȵĵ�����ת��Ϊ��������ϵ�µļ��ٶ�

% Rotate body accelerations to Earth frame
acc = quaternRotate([accX accY accZ], quaternConj(quat));

% Remove gravity from measurements unnecessary due to velocity integral drift compensation
acc = acc - [zeros(length(time), 2) ones(length(time), 1)];     

% Convert acceleration measurements to m/s/s
acc = acc * 9.81;

acc(:, 1) = acc(:, 1) - mean(acc(:, 1));
acc(:, 2) = acc(:, 2) - mean(acc(:, 2));
acc(:, 3) = acc(:, 3) - mean(acc(:, 3));

% Plot translational accelerations
figure('Position', [9 39 900 300], 'Number', 'off', 'Name', 'Accelerations');
hold on;
plot(time, acc(:,1), 'r'); plot(time, acc(:,2), 'g'); plot(time, acc(:,3), 'b');
title('Acceleration'); xlabel('Time (s)'); ylabel('Acceleration (m/s/s)');
legend('X', 'Y', 'Z');
hold off;

%% -------------------------------------------------------------------------
% ͨ�����ٶ�ת�������ٶ�
vel = zeros(size(acc));
for t = 2:length(vel)
    vel(t,:) = vel(t-1,:) + acc(t,:) * samplePeriod;
    % ��Ŀ�괦�ھ�ֹ״̬����Ϊ�ٶ�Ϊ0
    if(stationary(t) == 1)
        vel(t,:) = [0 0 0];     
    end
end

% Plot Current velocity
figure('Position', [9 39 900 300], 'Number', 'off', 'Name', 'Inter_velocity');
subplot(2,1,1);hold on;
plot(time, vel(:,1), 'r'); plot(time, vel(:,2), 'g'); plot(time, vel(:,3), 'b');
title('Inter_velocity'); xlabel('Time (s)'); ylabel('Inter_velocity (m/s)');
legend('X', 'Y', 'Z');
hold off;

% Compute integral drift during non-stationary periods
velDrift        = zeros(size(vel));
stationaryStart = find([0; diff(stationary)] == -1);%��ʼ���뾲ֹʱ��
stationaryEnd   = find([0; diff(stationary)] == 1); %��ʼ�����˶�ʱ��

for i = 1:numel(stationaryEnd)
    driftRate = vel(stationaryEnd(i)-1, :) / (stationaryEnd(i) - stationaryStart(i));
    enum      = 1:(stationaryEnd(i) - stationaryStart(i));
    drift     = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
   
    velDrift(stationaryStart(i):stationaryEnd(i)-1, :) = drift;
end

% Remove integral drift
vel = vel - velDrift;

% Plot translational velocity
subplot(2,1,2);hold on;
plot(time, vel(:,1), 'r');plot(time, vel(:,2), 'g');plot(time, vel(:,3), 'b');
title('Velocity');xlabel('Time (s)');ylabel('Velocity (m/s)');
legend('X', 'Y', 'Z');
hold off;
%vel = vel + velDrift;

% -------------------------------------------------------------------------
% Compute translational position Integrate velocity to yield position
pos = zeros(size(vel));
for t = 2:length(pos)
    pos(t,:) = pos(t-1,:) + vel(t,:) * samplePeriod;
end

% Plot translational position
figure('Position', [9 39 900 600], 'Number', 'off', 'Name', 'Position');
hold on;
plot(time, pos(:,1), 'r');plot(time, pos(:,2), 'g');plot(time, pos(:,3), 'b');
title('Position');xlabel('Time (s)');ylabel('Position (m)');
legend('X', 'Y', 'Z');
hold off;

%% ����Ƶ���˲� �ܹ�ȥ��������ĸ���
% linVelF(:, 1) = integral_freq(vel(:, 1), 0.21, 100, 1/samplePeriod,1);
% linVelF(:, 2) = integral_freq(vel(:, 2), 0.21, 100, 1/samplePeriod,1);
% linVelF(:, 3) = integral_freq(vel(:, 3), 0.21, 100, 1/samplePeriod,1);
% 
% figure('Number', 'off', 'Name', 'High-pass filtered Linear Velocity');hold on;
% plot(linVelF(:,1), 'r');plot(linVelF(:,2), 'g');plot(linVelF(:,3), 'b');
% xlabel('sample');ylabel('g');title('High-pass filtered linear velocity');
% legend('X', 'Y', 'Z');

% -------------------------------------------------------------------------
% Plot 3D foot trajectory

% Remove stationary periods from data to plot
% posPlot = pos(find(~stationary), :);
% quatPlot = quat(find(~stationary), :);
% posPlot = pos;
% quatPlot = quat;
% 
% % Extend final sample to delay end of animation
% extraTime = 20;
% onesVector = ones(extraTime*(1/samplePeriod), 1);
% posPlot = [posPlot; [posPlot(end, 1)*onesVector, posPlot(end, 2)*onesVector, posPlot(end, 3)*onesVector]];
% quatPlot = [quatPlot; [quatPlot(end, 1)*onesVector, quatPlot(end, 2)*onesVector, quatPlot(end, 3)*onesVector, quatPlot(end, 4)*onesVector]];
% 
% % Create 6 DOF animation
% SamplePlotFreq = 4;
% Spin = 120;
% SixDOFanimation2(posPlot, quatern2rotMat(quatPlot), ...
%                  'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
%                  'Position', [9 39 1280 768], 'View', [(100:(Spin/(length(posPlot)-1)):(100+Spin))', 10*ones(length(posPlot), 1)], ...
%                  'AxisLength', 0.1, 'ShowArrowHead', false, ...
%                  'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
%                  'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));
