% 利用IMU的6个自由度的信息，通过各种约束进姿态和位移的求解
clear;close all;clc;
addpath('quaternion_library'); addpath('ximu_matlab_library'); 
addpath('src');                addpath('AHRS');

% 数据的导入
expt_data = xlsread('LoggedData/data1.xlsx','data2');
base_data = xlsread('LoggedData/data1.xlsx','base2');

% 数据的清零操作 去除偏移的干扰
gyr       = expt_data(:, 5:7) / 15;            acc      = expt_data(:, 2:4) / 1000;
gyr_bias  = mean(base_data(:, 5:7) /   15, 1); acc_bias = mean(base_data(:, 2:4) / 1000, 1);
time      = expt_data(:, 1);   sampleFreq = 100; initFrame = 300;

% 数据的预处理操作
[AccData, GyrData] = data_preprocess(acc, gyr, 1, acc_bias, gyr_bias);

%运动时刻与静止时刻的检测
[staticMatix, MotionMatix] = static_detect(AccData, initFrame, sampleFreq, 0.05, 100);

%每一时刻姿态的求解
[EuleAngle, RotMatix] = cal_ery_attitude(AccData, GyrData, 1 / sampleFreq, initFrame, MotionMatix);

% 加速度的修正与计算
ProjAccData = project_accelerate( AccData, RotMatix, MotionMatix);

% 进行速度的求解
velData     = cal_object_Velocity(ProjAccData, MotionMatix, sampleFreq, 'Time');

% 进行位移的求解
PosData     = cal_object_Position(velData, MotionMatix, sampleFreq, 'Time');

PosData     = zeros(size(PosData));
%进行姿态的显示 看看姿态是否准确
SamplePlotFreq = 50;
SixDOFanimation1(PosData, RotMatix, ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
                'Position', [9 39 1280 720], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', (sampleFreq / SamplePlotFreq));   