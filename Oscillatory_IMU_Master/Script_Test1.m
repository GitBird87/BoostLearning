%% Housekeeping
% See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

addpath('ximu_matlab_library');	% include x-IMU MATLAB library
addpath('quaternion_library');	% include quatenrion library
close all; clear; clc;
 
%% 数据的导入
samplePeriod = 1/100;   
exp_data     = xlsread('Datasets/capulse_data.xlsx','base1_data');
base_data    = xlsread('Datasets/capulse_data.xlsx','base1');

gyr      = exp_data(:,  5:7) / 15; acc      = exp_data(:,  2:4) / 1000;
gyr_base = base_data(:, 5:7) / 15; acc_base = base_data(:, 2:4) / 1000;

for i = 1:3
    gyr(:, i) = gyr(:, i) - mean(gyr_base(:, i));
    acc(:, i) = acc(:, i) - mean(acc_base(:, i));
end
acc(:, 3) = acc(:, 3) + 1;

%% 基础数据的滤波与绘制
%
figure('Number', 'off', 'Name', 'Gyroscope');
subplot(2,1,1); hold on;xlabel('sample'); ylabel('dps');
plot(gyr(:,1), 'r');plot(gyr(:,2), 'g');plot(gyr(:,3), 'b');
title('Gyroscope');legend('X', 'Y', 'Z');

% 进行滤波
for i = 1:3
    gyr(:, i) = kalman(gyr(:, i), 0, 4e-4, 0.05);
    %gyr(:, i) = medfilt1(gyr(:, i), 3);
end
subplot(2,1,2); hold on;xlabel('sample'); ylabel('dps');
plot(gyr(:,1), 'r');plot(gyr(:,2), 'g');plot(gyr(:,3), 'b');
title('GyroscopeK');legend('X', 'Y', 'Z');

%
figure('Number', 'off', 'Name', 'Accelerometer');
subplot(2,1,1); hold on;xlabel('sample');ylabel('g');
plot(acc(:,1), 'r');plot(acc(:,2), 'g');plot(acc(:,3), 'b');
title('Accelerometer');legend('X', 'Y', 'Z');
% 滤波
for i = 1:3
    if i == 3
       acc(:, i) = kalman(acc(:, i), 1, 4e-4, 0.003);
    else
       acc(:, i) = kalman(acc(:, i), 0, 4e-4, 0.003); 
    end
    %acc(:, i) = medfilt1(acc(:, i), 3);
end
subplot(2,1,2);hold on;xlabel('sample');ylabel('g');
plot(acc(:,1), 'r');plot(acc(:,2), 'g');plot(acc(:,3), 'b');
title('AccelerometerK');legend('X', 'Y', 'Z');

%% 利用加速度和陀螺仪的角度信息计算每个位置相对于机体坐标系的旋转矩阵R 没有位移分量
RotMatix = zeros(3,3,length(gyr));                            % rotation matrix describing sensor relative to Earth
ahrs     = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1); %陀螺仪的值为 角度/周期
%ahrs     = MadgwickAHRS('SamplePeriod', samplePeriod, 'Beta', 0.1);
for i = 1:length(gyr)
    %陀螺仪的输入为弧度 根据陀螺仪和加速计的值，计算当前的四元数
    ahrs.UpdateIMU(gyr(i,:) * (pi/180), acc(i,:));   
    %将四元数转化为旋转矩阵
    RotMatix(:,:,i) = quatern2rotMat(ahrs.Quaternion)'; 
end

%% 根据计算出的旋转角度进行加速度的修正
tcAcc = zeros(size(acc));  % accelerometer in Earth frame
for i = 1:length(acc)
    tcAcc(i,:) = RotMatix(:,:,i) * acc(i,:)';
end

% Plot
figure('Number', 'off', 'Name', '''Tilt-Compensated'' accelerometer');
hold on;
plot(tcAcc(:,1), 'r');plot(tcAcc(:,2), 'g');plot(tcAcc(:,3), 'b');
xlabel('sample');ylabel('g');title('''Tilt-compensated'' accelerometer');
legend('X', 'Y', 'Z');

%% 去除重力加速度的影响 计算线性加速度 单位转化为m/s/s
linAcc = tcAcc - [zeros(length(tcAcc), 1), zeros(length(tcAcc), 1), ones(length(tcAcc), 1)];
linAcc = linAcc * 9.81;     % convert from 'g' to m/s/s
linAcc(:, 1) = linAcc(:, 1) - mean(linAcc(:, 1));
linAcc(:, 2) = linAcc(:, 2) - mean(linAcc(:, 2));
linAcc(:, 3) = linAcc(:, 3) - mean(linAcc(:, 3));

% Plot
figure('Number', 'off', 'Name', 'Linear Acceleration');
hold on;
plot(linAcc(:,1), 'r');plot(linAcc(:,2), 'g');plot(linAcc(:,3), 'b');
xlabel('sample');ylabel('g');title('Linear acceleration');
legend('X', 'Y', 'Z');

%% 对加速度积分得到线性速度
linVel = zeros(size(linAcc));
for i = 2:length(linAcc)
    linVel(i,:) = linVel(i-1,:) + linAcc(i,:) * samplePeriod;
end

% Plot
figure('Number', 'off', 'Name', 'Linear Velocity');
hold on;
plot(linVel(:,1), 'r');plot(linVel(:,2), 'g');plot(linVel(:,3), 'b');
xlabel('sample');ylabel('g');title('Linear velocity');
legend('X', 'Y', 'Z');

%% 对线性速度进行高通滤波，去除漂移的影响
order = 5;filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linVelHP = filtfilt(b, a, linVel);

% Plot
figure('Number', 'off', 'Name', 'High-pass filtered Linear Velocity');
hold on;
plot(linVelHP(:,1), 'r');plot(linVelHP(:,2), 'g');plot(linVelHP(:,3), 'b');
xlabel('sample');ylabel('g');title('High-pass filtered linear velocity');
legend('X', 'Y', 'Z');

%% 对速度进行积分得到相对位移
linPos = zeros(size(linVelHP));
for i = 2:length(linVelHP)
    linPos(i,:) = linPos(i-1,:) + linVelHP(i,:) * samplePeriod;
end

% Plot
figure('Number', 'off', 'Name', 'Linear Position');
hold on;
plot(linPos(:,1), 'r');plot(linPos(:,2), 'g');plot(linPos(:,3), 'b');
xlabel('sample');ylabel('g');
title('Linear position');legend('X', 'Y', 'Z');

%% 对位移进行高通滤波 去除漂移的影响
order = 5;filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linPosHP = filtfilt(b, a, linPos);

% Plot
figure('Number', 'off', 'Name', 'High-pass filtered Linear Position');
hold on;
plot(linPosHP(:,1), 'r');plot(linPosHP(:,2), 'g');plot(linPosHP(:,3), 'b');
xlabel('sample');ylabel('g');title('High-pass filtered linear position');
legend('X', 'Y', 'Z');

%% 进行效果的显示
SamplePlotFreq = 20;
SixDOFanimation1(linPosHP, RotMatix, ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
                'Position', [9 39 1280 720], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));            
 
%% End of script