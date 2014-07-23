%%

%datashift('base1_data.txt', 1 / 400, 'Datasets/data2.xlsx','base1_data');%���ַ���ʽ���ļ�
%datashift('base1.txt',      1 / 400, 'Datasets/data2.xlsx','base1');     %���ַ���ʽ���ļ�

%% ����IMU��6�����ɶȵ���Ϣ��ͨ������Լ������̬��λ�Ƶ����

clear;close all;clc;

addpath('quaternion_library');
addpath('ximu_matlab_library');
addpath('tools');

%% ���ݵĵ���
exp_data  = xlsread('Datasets/capulse_data.xlsx','data2');
gyr       = exp_data(:,  5:7) / 15; acc          = exp_data(:,  2:4) / 1000;
time      = exp_data(:, 1) / 100;   samplePeriod = 1/10;
acc(:, 2) = 0 - acc(:, 2);          acc(:, 3)    = 0 - acc(:, 3);

%% Kalman�˲�����
gyrK = zeros(size(gyr)); accK = zeros(size(acc));
for i = 1:3
    gyrK(:, i) = kalman(gyr(:, i), 0, 4e-4, 0.03);
    if i == 3
        accK(:, i) = kalman(acc(:, i), 1, 4e-4, 0.03);
    else
        accK(:, i) = kalman(acc(:, i), 0, 4e-4, 0.03);
    end
end

%% �������ݵ��˲������
figure('Number', 'off', 'Name', 'Gyroscope');
subplot(2,1,1); hold on; xlabel('sample'); ylabel('dps');
plot(gyr(:,1), 'r');plot(gyr(:,2), 'g');plot(gyr(:,3), 'b');title('Gyroscope');legend('X', 'Y', 'Z');
subplot(2,1,2); hold on; xlabel('sample'); ylabel('dps');
plot(gyrK(:,1), 'r');plot(gyrK(:,2), 'g');plot(gyrK(:,3), 'b');title('GyroscopeK');legend('X', 'Y', 'Z');

figure('Number', 'off', 'Name', 'Accelerometer');
subplot(2,1,1); hold on;xlabel('sample');ylabel('g');
plot(acc(:,1), 'r');plot(acc(:,2), 'g');plot(acc(:,3), 'b');title('Accelerometer');legend('X', 'Y', 'Z');
subplot(2,1,2); hold on;xlabel('sample');ylabel('g');
plot(accK(:,1), 'r');plot(accK(:,2), 'g');plot(accK(:,3), 'b');title('AccelerometerK');legend('X', 'Y', 'Z');

%% ������̬�����
RotMatix = zeros(3,3,length(gyr));                            % rotation matrix describing sensor relative to Earth
ahrs     = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1); %�����ǵ�ֵΪ �Ƕ�/����
%ahrs     = MadgwickAHRS('SamplePeriod', samplePeriod, 'Beta', 0.1);
for i = 1:length(gyr)
    ahrs.UpdateIMU(gyr(i,:) * (pi/180), accK(i,:));            %���������뻡�� ������&���ټ�->��Ԫ�� 
    RotMatix(:,:,i) = quatern2rotMat(ahrs.Quaternion)';       %��Ԫ��ת��Ϊ��ת����
end

%% ���ٶȵ�����
tcAcc = zeros(size(acc));  % accelerometer in Earth frame
for i = 1:length(acc)
    tcAcc(i,:) = RotMatix(:,:,i) * accK(i,:)';
end

% Plot
figure('Number', 'off', 'Name', '''Tilt-Compensated'' accelerometer');
subplot(3,1,1); hold on;plot(tcAcc(:,1), 'r');plot(accK(:,1), 'g');
subplot(3,1,2); hold on;plot(tcAcc(:,2), 'r');plot(accK(:,2), 'g');
subplot(3,1,3); hold on;plot(tcAcc(:,3), 'r');plot(accK(:,3), 'g');

%% ��̬λ�õ�Ѱ��
stationary(:, 1) = zeros_amend(abs(tcAcc(:, 1)),     0.05, 10);
stationary(:, 2) = zeros_amend(abs(tcAcc(:, 2)),     0.05, 10);
stationary(:, 3) = zeros_amend(abs(tcAcc(:, 3) - 1), 0.05, 10);

tcAcc  = tcAcc - [zeros(length(time), 2) ones(length(time), 1)]; 

%% ���м��ٶȵĵ���
for i = 1: 3
    statStart{i} = find([0; diff(stationary(:, i))] == -1); %�Ӿ�ֹ���˶�״̬ 
    statEnd{i}   = find([0; diff(stationary(:, i))] ==  1); %���˶�����ֹ״̬
end

% ��Ŀ�괦�ھ�ֹ״̬����Ϊ�ٶ�Ϊ0
for t = 2:length(tcAcc)
    if(stationary(t, 1) == 1), tcAcc(t, 1) = 0; end
    if(stationary(t, 2) == 1), tcAcc(t, 2) = 0; end 
    if(stationary(t, 3) == 1), tcAcc(t, 3) = 0; end 
    
end

for i = 1:3
    for j = 1 : min(numel(statStart{i}), numel(statEnd{i}))
         mean_acc = mean(tcAcc(statStart{i}(j) : statEnd{i}(j), i));
         tcAcc(statStart{i}(j) : statEnd{i}(j), i) = tcAcc(statStart{i}(j) : statEnd{i}(j), i) - mean_acc;
    end
end

figure('Position', [9 39 900 600], 'Number', 'off', 'Name', 'Sensor Data');
subplot(3,1,1);hold on;plot(time, tcAcc(:, 1), 'r');plot(time, stationary(:, 1), 'k');title('AccX');
subplot(3,1,2);hold on;plot(time, tcAcc(:, 2), 'r');plot(time, stationary(:, 2), 'k');title('AccY');
subplot(3,1,3);hold on;plot(time, tcAcc(:, 3), 'r');plot(time, stationary(:, 3), 'k');title('AccZ');

tcAcc  = tcAcc * 9.81;

%% �����ٶȵ�ʱ�����
linVel(:, 1) = integral_time(tcAcc(:, 1), samplePeriod, 3);
linVel(:, 2) = integral_time(tcAcc(:, 2), samplePeriod, 3);
linVel(:, 3) = integral_time(tcAcc(:, 3), samplePeriod, 3);

% Plot translational velocity
figure('Position', [9 39 900 300], 'Number', 'off', 'Name', 'Velocity');
subplot(2,1,1); hold on; plot(time, linVel(:,1), 'r');plot(time, linVel(:,2), 'g');plot(time, linVel(:,3), 'b');
title('Velocity');xlabel('Time (s)');ylabel('Velocity (m/s)');legend('X', 'Y', 'Z');


%% ���м��ٶȵ�Ƶ�����
% linVelF(:, 1) = integral_freq(tcAcc(:, 1), 0.31, 30, 1/samplePeriod,1);
% linVelF(:, 2) = integral_freq(tcAcc(:, 2), 0.31, 30, 1/samplePeriod,1);
% linVelF(:, 3) = integral_freq(tcAcc(:, 3), 0.31, 30, 1/samplePeriod,1);
% 
% % Plot translational velocity
% subplot(2,1,2); hold on; plot(time, linVelF(:,1), 'r');plot(time, linVelF(:,2), 'g');plot(time, linVelF(:,3), 'b');
% title('VelocityF');xlabel('Time (s)');ylabel('Velocity (m/s)');legend('X', 'Y', 'Z');



%% �����ٶȵ�������




%% ����λ�ƵĻ���
linPos(:, 1) = integral_time(linVel(:, 1), samplePeriod, 3);
linPos(:, 2) = integral_time(linVel(:, 2), samplePeriod, 3);
linPos(:, 3) = integral_time(linVel(:, 3), samplePeriod, 3);

% Plot
figure('Number', 'off', 'Name', 'Position');hold on;
plot(time, linPos(:,1), 'r');plot(time, linPos(:,2), 'g');plot(time, linPos(:,3), 'b');title('position');
legend('X', 'Y', 'Z');

linPos = zeros(length(linPos), 3);

%% ������̬����ʾ ������̬�Ƿ�׼ȷ
SamplePlotFreq = 1;
SixDOFanimation1(linPos, RotMatix, ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
                'Position', [9 39 1280 720], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
                'CreateAVI', true, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));   