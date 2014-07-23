clear;close all;clc;

addpath('quaternion_library');
addpath('ximu_matlab_library');

%% -------------------------------------------------------------------------
% Select dataset (comment in/out)
exp_data   = xlsread('Datasets/data1.xlsx','base1_data');
base_data  = xlsread('Datasets/data1.xlsx','base1');
initPeriod = 1;

gyr      = exp_data(:, 5:7)  / 15; acc      = exp_data(:, 2:4)  / 1000;
gyr_base = base_data(:, 5:7) / 15; acc_base = base_data(:, 2:4) / 1000;

gyrX = gyr(:, 1) - mean(gyr_base(:, 1));
gyrY = gyr(:, 2) - mean(gyr_base(:, 2));
gyrZ = gyr(:, 3) - mean(gyr_base(:, 3));
accX = acc(:, 1) - mean(acc_base(:, 1));
accY = acc(:, 2) - mean(acc_base(:, 2));
accZ = acc(:, 3) - mean(acc_base(:, 3)) + 1;

% gyrX = gyr(:, 1); gyrY = gyr(:, 2); gyrZ = gyr(:, 3);
% accX = acc(:, 1); accY = acc(:, 2); accZ = acc(:, 3);

%转化为s
time         = exp_data(:, 1);
samplePeriod = 1/100;

clear gyr acc exp_data base_data

%% -------------------------------------------------------------------------
% step2: 设置处理的帧数 起始位置和终止位置
startTime = time(1); stopTime  = time(end);
indexSel = find(sign(time-startTime)+1, 1) : find(sign(time-stopTime)+1, 1);
time     = time(indexSel);

gyrX = gyrX(indexSel, :); gyrY = gyrY(indexSel, :); gyrZ = gyrZ(indexSel, :);
accX = accX(indexSel, :); accY = accY(indexSel, :); accZ = accZ(indexSel, :);

%% -------------------------------------------------------------------------
% step3: 对输入的数据进行滤波分析

% 滤波前的数据
figure('Number', 'off', 'Name', 'Accelerometer');
subplot(2,1,1);hold on; plot(accX, 'r');plot(accY, 'g');plot(accZ, 'b');
xlabel('sample');ylabel('g'); title('Accelerometer');legend('X', 'Y', 'Z');

% kalman滤波
accX = kalman(accX, 0, 4e-4, 0.05); accY = kalman(accY, 0, 4e-4, 0.05); accZ = kalman(accZ, 1, 4e-4, 0.05);
% 中值滤波
% accX = medfilt1(accX, 3); accY = medfilt1(accY, 3); accZ = medfilt1(accZ, 3);

% 滤波后的数据
subplot(2,1,2); hold on; plot(accX, 'r');plot(accY, 'g');plot(accZ, 'b');
xlabel('sample');ylabel('g');title('AccelerometerK');legend('X', 'Y', 'Z');

% 滤波前的数据
figure('Number', 'off', 'Name', 'Gyroscope');
subplot(2,1,1);hold on; plot(gyrX, 'r');plot(gyrY, 'g');plot(gyrZ, 'b');
xlabel('sample');ylabel('g'); title('Gyroscope');legend('X', 'Y', 'Z');

% kalman滤波
gyrX = kalman(gyrX, 0, 4e-4, 0.05);gyrY = kalman(gyrY, 0, 4e-4, 0.05);gyrZ = kalman(gyrZ, 1, 4e-4, 0.05);
% 中值滤波
% gyrX = medfilt1(gyrX, 3); gyrY = medfilt1(gyrY, 3); gyrZ = medfilt1(gyrZ, 3);

% 滤波后的数据
subplot(2,1,2); hold on;plot(gyrX, 'r');plot(gyrY, 'g');plot(gyrZ, 'b');
xlabel('sample');ylabel('g');title('GyroscopeK');legend('X', 'Y', 'Z');

%% -------------------------------------------------------------------------
% step4: 静态帧数的检测
% filtCutOff  = 0.001;
% [b, a]      = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
% acc_magX    = abs(filtfilt(b, a, abs(accX)));
% acc_magY    = abs(filtfilt(b, a, abs(accY)));
% acc_magZ    = abs(filtfilt(b, a, abs(accZ)));
% filtCutOff  = 6;
% [b, a]      = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
% acc_magFltX = filtfilt(b, a, acc_magX);
% acc_magFltY = filtfilt(b, a, acc_magY);
% acc_magFltZ = filtfilt(b, a, acc_magZ);

% Threshold detection
stationaryX = zeros_amend(abs(accX),     0.05, 100);
stationaryY = zeros_amend(abs(accY),     0.05, 100);
stationaryZ = zeros_amend(abs(accZ - 1), 0.05, 100);

%% -------------------------------------------------------------------------
% step5: 绘制原始的数据和静态时刻的数据

figure('Position', [9 39 900 600], 'Number', 'off', 'Name', 'Sensor Data');
ax(1) = subplot(3,1,1);
    hold on;
    plot(time, accX, 'r');plot(time, stationaryX, 'k', 'LineWidth', 2);%plot(time, acc_magFltX, ':k');
    title('AccelerometerX');xlabel('Time (s)');ylabel('Acceleration (g)');
    legend('X', 'Filtered', 'Stationary');
    hold off;
ax(2) = subplot(3,1,2);
    hold on;
    plot(time, accY, 'r');plot(time, stationaryY, 'k', 'LineWidth', 2);%plot(time, acc_magFltY, ':k');
    title('AccelerometerY');xlabel('Time (s)');ylabel('Acceleration (g)');
    legend('Y', 'Filtered', 'Stationary');
    hold off;
ax(3) = subplot(3,1,3);
    hold on;
    plot(time, accZ, 'r');plot(time, stationaryZ, 'k', 'LineWidth', 2);%plot(time, acc_magFltZ, ':k');
    title('AccelerometerZ');xlabel('Time (s)');ylabel('Acceleration (g)');
    legend('Z', 'Filtered', 'Stationary');
    hold off;
linkaxes(ax,'x');

%% -------------------------------------------------------------------------
% step6: 进行角度的计算
quat = zeros(length(time), 4);
AHRSalgorithm = AHRS('SamplePeriod', samplePeriod, 'Kp', 1, 'KpInit', 1);

% 寻找目标初始化状态的时间段 然后在此时间段反复迭代寻找初始时刻的四元数
indexSel = 1 : find( sign( time-( time(1) + initPeriod ) ) + 1, 1);
for i = 1:2000
    AHRSalgorithm.UpdateIMU([0 0 0], [mean(accX(indexSel)) mean(accY(indexSel)) mean(accZ(indexSel))]);
end

% 对所有的元素进行四元数计算
for t = 1:length(time) 
    % 目标处于静态时刻才进行重力下的约束
    %if(stationaryX(t) && stationaryY(t) && stationaryZ(t))
        AHRSalgorithm.Kp = 0.5;
    %else
    %    AHRSalgorithm.Kp = 0;
    %end 
    AHRSalgorithm.UpdateIMU(deg2rad([gyrX(t) gyrY(t) gyrZ(t)]), [accX(t) accY(t) accZ(t)]);
    quat(t,:) = AHRSalgorithm.Quaternion;
end

%% -------------------------------------------------------------------------
% step7: 进行加速度的调整，转化为地球坐标系下的加速度

% Rotate body accelerations to Earth frame
acc = quaternRotate([accX accY accZ], quaternConj(quat));
acc = acc - [zeros(length(time), 2) ones(length(time), 1)];     
acc = acc * 9.81;

acc(:, 1) = acc(:, 1) - mean(acc(:, 1));
acc(:, 2) = acc(:, 2) - mean(acc(:, 2));
acc(:, 3) = acc(:, 3) - mean(acc(:, 3));

% Plot translational accelerations
figure('Position', [9 39 900 300], 'Number', 'off', 'Name', 'Accelerations'); hold on;
plot(time, acc(:,1), 'r');plot(time, acc(:,2), 'g');plot(time, acc(:,3), 'b');
title('Acceleration');xlabel('Time (s)');ylabel('Acceleration (m/s/s)');
legend('X', 'Y', 'Z'); hold off;

%% -------------------------------------------------------------------------
% step8: 通过加速度转换计算速度
vel = zeros(size(acc));
for t = 2:length(vel)
    vel(t,:) = vel(t-1,:) + acc(t,:) * samplePeriod;  
    % 当目标处于静止状态，认为速度为0
    if(stationary(t) == 1)
        vel(t,:) = [0 0 0];     
    end 
end

% Compute integral drift during non-stationary periods
% 加速度为0的时候速度应该也为0
velDrift        = zeros(size(vel));
stationaryStart = find([0; diff(stationary)] == -1); %从静止到运动状态 
stationaryEnd   = find([0; diff(stationary)] ==  1); %从运动到静止状态

for i = 1:min(numel(stationaryEnd), numel(stationaryStart))
    driftRate = vel(stationaryEnd(i)-1, :) / (stationaryEnd(i) - stationaryStart(i));
    enum      = 1:(stationaryEnd(i) - stationaryStart(i));
    drift     = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
  
    velDrift(stationaryStart(i):stationaryEnd(i)-1, :) = drift;
end

% Plot translational velocity
figure('Position', [9 39 900 300], 'Number', 'off', 'Name', 'Velocity');
subplot(2,1,1); hold on;
plot(time, vel(:,1), 'r');plot(time, vel(:,2), 'g');plot(time, vel(:,3), 'b');
title('Velocity');xlabel('Time (s)');ylabel('Velocity (m/s)');legend('X', 'Y', 'Z');
hold off;

vel = vel - velDrift;

subplot(2,1,2); hold on;
plot(time, vel(:,1), 'r');plot(time, vel(:,2), 'g');plot(time, vel(:,3), 'b');
title('VelocityF');xlabel('Time (s)');ylabel('Velocity (m/s)');legend('X', 'Y', 'Z');
hold off;

%% 对速度进行滤波
% order = 2;filtCutOff = 0.2;
% [b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
% vel(:,1) = filtfilt(b, a, vel(:,1));
% 
% order = 2;filtCutOff = 0.2;
% [b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
% vel(:,2) = filtfilt(b, a, vel(:,2));
% 
% order = 2;filtCutOff = 0.1;
% [b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
% vel(:,3) = filtfilt(b, a, vel(:,3));
% 
% % step8: 通过加速度转换计算速度
% for t = 2:length(vel)
%     % 当目标处于静止状态，认为速度为0
%     if(stationary(t) == 1)
%         vel(t,:) = [0 0 0];     
%     end
% end
% 
% % Plot translational velocity
% figure('Position', [9 39 900 300], 'Number', 'off', 'Name', 'VelocityF');hold on;
% plot(time, vel(:,1), 'r');plot(time, vel(:,2), 'g');plot(time, vel(:,3), 'b');
% title('VelocityF');xlabel('Time (s)');ylabel('Velocity (m/s)');legend('X', 'Y', 'Z');
% hold off;

%% -------------------------------------------------------------------------
% 进行位移的计算
pos = zeros(size(vel));
for t = 2:length(pos)
    pos(t,:) = pos(t-1,:) + vel(t,:) * samplePeriod;    % integrate velocity to yield position
end

% Plot translational position
figure('Position', [9 39 900 600], 'Number', 'off', 'Name', 'Position');
hold on;
plot(time, pos(:,1), 'r');plot(time, pos(:,2), 'g');plot(time, pos(:,3), 'b');
title('Position');xlabel('Time (s)');ylabel('Position (m)');legend('X', 'Y', 'Z');
hold off;

%% 对位移进行滤波
% order = 2;filtCutOff = 0.1; 
% [b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
% posF(:,1) = filtfilt(b, a, pos(:,1));
% 
% order = 2;filtCutOff = 0.1;
% [b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
% posF(:,2) = filtfilt(b, a, pos(:,2));
% 
% order = 1;filtCutOff = 3;
% [b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'low');
% posF(:,3) = filtfilt(b, a, pos(:,3));
% 
% % Plot translational position
% figure('Position', [9 39 900 600], 'Number', 'off', 'Name', 'PositionF');
% hold on;
% plot(time, posF(:,1), 'r');plot(time, posF(:,2), 'g');plot(time, posF(:,3), 'b');
% title('PositionF');xlabel('Time (s)');ylabel('Position (m)');legend('X', 'Y', 'Z');
% hold off;

% %% 对位置进行频域分析
% accFlt(:, 1) = signalfft(acc(:,1), 15, 0, 'X');
% accFlt(:, 2) = signalfft(acc(:,2), 15, 0, 'Y');
% accFlt(:, 3) = signalfft(acc(:,3), 15, 0, 'Z');
% 
% % Plot translational position
% figure('Position', [9 39 900 600], 'Number', 'off', 'Name', 'PositionF');
% hold on;
% plot(time, accFlt(:,1), 'r');
% plot(time, accFlt(:,2), 'g');
% plot(time, accFlt(:,3), 'b');
% title('Position');xlabel('Time (s)');ylabel('Position (m)');legend('X', 'Y', 'Z');
% hold off;

% -------------------------------------------------------------------------
% Plot 3D foot trajectory

% % Remove stationary periods from data to plot
% posPlot = pos(find(~stationary), :);
% quatPlot = quat(find(~stationary), :);
posPlot = pos;
quatPlot = quat;

% Extend final sample to delay end of animation
extraTime = 20;
onesVector = ones(extraTime*(1/samplePeriod), 1);
posPlot = [posPlot; [posPlot(end, 1)*onesVector, posPlot(end, 2)*onesVector, posPlot(end, 3)*onesVector]];
quatPlot = [quatPlot; [quatPlot(end, 1)*onesVector, quatPlot(end, 2)*onesVector, quatPlot(end, 3)*onesVector, quatPlot(end, 4)*onesVector]];


SamplePlotFreq = 10;
SixDOFanimation1(posPlot, quatern2rotMat(quatPlot), ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
                'Position', [9 39 1280 720], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));     

% % Create 6 DOF animation
% SamplePlotFreq = 4;
% Spin = 120;
% SixDOFanimation2(posPlot, quatern2rotMat(quatPlot), ...
%                 'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
%                 'Position', [9 39 1280 768], 'View', [(100:(Spin/(length(posPlot)-1)):(100+Spin))', 10*ones(length(posPlot), 1)], ...
%                 'AxisLength', 0.1, 'ShowArrowHead', false, ...
%                 'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
%                 'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));

