% See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

%% Housekeeping
 
addpath('ximu_matlab_library');	% include x-IMU MATLAB library
addpath('quaternion_library');	% include quatenrion library
addpath('tools');
close all;clear;clc;
 
%% ���ݵĵ���

%ԭʼ����
xIMUdata     = xIMUdataClass('LoggedData/LoggedData');
samplePeriod = 1/256;

gyr = [xIMUdata.CalInertialAndMagneticData.Gyroscope.X...
       xIMUdata.CalInertialAndMagneticData.Gyroscope.Y...
       xIMUdata.CalInertialAndMagneticData.Gyroscope.Z];        % gyroscope
acc = [xIMUdata.CalInertialAndMagneticData.Accelerometer.X...
       xIMUdata.CalInertialAndMagneticData.Accelerometer.Y...
       xIMUdata.CalInertialAndMagneticData.Accelerometer.Z];	% accelerometer

% Plot
figure('Number', 'off', 'Name', 'Gyroscope');hold on;
plot(gyr(:,1), 'r');plot(gyr(:,2), 'g');plot(gyr(:,3), 'b');
xlabel('sample'); ylabel('dps');title('Gyroscope');legend('X', 'Y', 'Z');

figure('Number', 'off', 'Name', 'Accelerometer');hold on;
plot(acc(:,1), 'r');plot(acc(:,2), 'g');plot(acc(:,3), 'b');
xlabel('sample');ylabel('g');title('Accelerometer');legend('X', 'Y', 'Z');


%% Process data through AHRS algorithm (calcualte orientation)
% ���ü��ٶȺ������ǵĽǶ���Ϣ����ÿ��λ������ڻ�������ϵ����ת����R û��λ�Ʒ���

RotMatix = zeros(3,3,length(gyr));     % rotation matrix describing sensor relative to Earth
%ahrs    = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1);
ahrs     = MadgwickAHRS('SamplePeriod', samplePeriod, 'Beta', 0.1);

for i = 1:length(gyr)
    %�����ǵ�����Ϊ���� ���������Ǻͼ��ټƵ�ֵ�����㵱ǰ����Ԫ��
    ahrs.UpdateIMU(gyr(i,:) * (pi/180), acc(i,:));
    %����Ԫ��ת��Ϊ��ת���� ������ļ�����鱾�ϵĹ�ʽ��һ��
    RotMatix(:,:,i) = quatern2rotMat(ahrs.Quaternion)'; 
end

%% ���ݼ��������ת�ǶȽ��м��ٶȵ�����
tcAcc = zeros(size(acc));  % accelerometer in Earth frame
for i = 1:length(acc)
    tcAcc(i,:) = RotMatix(:,:,i) * acc(i,:)';
end

% Plot
figure('Number', 'off', 'Name', '''Tilt-Compensated'' accelerometer');hold on;
plot(tcAcc(:,1), 'r');plot(tcAcc(:,2), 'g');plot(tcAcc(:,3), 'b');
xlabel('sample');ylabel('g');title('''Tilt-compensated'' accelerometer');
legend('X', 'Y', 'Z');

%% ȥ���������ٶȵ�Ӱ�� �������Լ��ٶ� ��λת��Ϊm/s/s
linAcc = tcAcc - [zeros(length(tcAcc), 1), zeros(length(tcAcc), 1), ones(length(tcAcc), 1)];
linAcc = linAcc * 9.81;     % convert from 'g' to m/s/s

% Plot
figure('Number', 'off', 'Name', 'Linear Acceleration');hold on;
plot(linAcc(:,1), 'r');plot(linAcc(:,2), 'g');plot(linAcc(:,3), 'b');
xlabel('sample');ylabel('g');title('Linear acceleration');
legend('X', 'Y', 'Z');

%% �Լ��ٶȻ��ֵõ������ٶ�
linVel = zeros(size(linAcc));

for i = 2:length(linAcc)
    linVel(i,:) = linVel(i-1,:) + linAcc(i,:) * samplePeriod;
end

% Plot
figure('Number', 'off', 'Name', 'Linear Velocity');hold on;
plot(linVel(:,1), 'r');plot(linVel(:,2), 'g');plot(linVel(:,3), 'b');
xlabel('sample');ylabel('g');title('Linear velocity');
legend('X', 'Y', 'Z');

%% ����Ƶ���˲� �ܹ�ȥ��������ĸ���
% linVelF(:, 1) = integral_freq(linAcc(:, 1), 0.21, 30, 1/samplePeriod,1);
% linVelF(:, 2) = integral_freq(linAcc(:, 2), 0.21, 30, 1/samplePeriod,1);
% linVelF(:, 3) = integral_freq(linAcc(:, 3), 0.21, 30, 1/samplePeriod,1);
% 
% figure('Number', 'off', 'Name', 'High-pass filtered Linear Velocity');hold on;
% plot(linVelF(:,1), 'r');plot(linVelF(:,2), 'g');plot(linVelF(:,3), 'b');
% xlabel('sample');ylabel('g');title('High-pass filtered linear velocity');
% legend('X', 'Y', 'Z');

%% �������ٶȽ��и�ͨ�˲���ȥ��Ư�Ƶ�Ӱ��
order = 1;filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linVelHP = filtfilt(b, a, linVel);

% Plot
figure('Number', 'off', 'Name', 'High-pass filtered Linear Velocity');hold on;
plot(linVelHP(:,1), 'r');plot(linVelHP(:,2), 'g');plot(linVelHP(:,3), 'b');
xlabel('sample');ylabel('g');title('High-pass filtered linear velocity');
legend('X', 'Y', 'Z');

%% ���ٶȽ��л��ֵõ����λ��
linPos = zeros(size(linVelHP));

for i = 2:length(linVelHP)
    linPos(i,:) = linPos(i-1,:) + linVelHP(i,:) * samplePeriod;
end

% Plot
figure('Number', 'off', 'Name', 'Linear Position');hold on;
plot(linPos(:,1), 'r');plot(linPos(:,2), 'g');plot(linPos(:,3), 'b');
xlabel('sample');ylabel('g');title('Linear position');
legend('X', 'Y', 'Z');

%% ��λ�ƽ��и�ͨ�˲� ȥ��Ư�Ƶ�Ӱ��
order = 1;filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linPosHP = filtfilt(b, a, linPos);

% Plot
figure('Number', 'off', 'Name', 'High-pass filtered Linear Position');hold on;
plot(linPosHP(:,1), 'r');plot(linPosHP(:,2), 'g');plot(linPosHP(:,3), 'b');
xlabel('sample');ylabel('g');title('High-pass filtered linear position');
legend('X', 'Y', 'Z');

%% ����Ч������ʾ
% SamplePlotFreq = 20;
% SixDOFanimation(linPosHP, RotMatix, ...
%                 'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
%                 'Position', [9 39 1280 720], ...
%                 'AxisLength', 0.1, 'ShowArrowHead', false, ...
%                 'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
%                 'CreateAVI', true, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));            
 
%% End of script