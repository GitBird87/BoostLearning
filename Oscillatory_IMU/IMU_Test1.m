% ����IMU��6�����ɶȵ���Ϣ��ͨ������Լ������̬��λ�Ƶ����
clear;close all;clc;
addpath('quaternion_library'); addpath('ximu_matlab_library'); 
addpath('src');                addpath('AHRS');

% ���ݵĵ���
expt_data = xlsread('LoggedData/data1.xlsx','data2');
base_data = xlsread('LoggedData/data1.xlsx','base2');

% ���ݵ�������� ȥ��ƫ�Ƶĸ���
gyr       = expt_data(:, 5:7) / 15;            acc      = expt_data(:, 2:4) / 1000;
gyr_bias  = mean(base_data(:, 5:7) /   15, 1); acc_bias = mean(base_data(:, 2:4) / 1000, 1);
time      = expt_data(:, 1);   sampleFreq = 100; initFrame = 300;

% ���ݵ�Ԥ�������
[AccData, GyrData] = data_preprocess(acc, gyr, 1, acc_bias, gyr_bias);

%�˶�ʱ���뾲ֹʱ�̵ļ��
[staticMatix, MotionMatix] = static_detect(AccData, initFrame, sampleFreq, 0.05, 100);

%ÿһʱ����̬�����
[EuleAngle, RotMatix] = cal_ery_attitude(AccData, GyrData, 1 / sampleFreq, initFrame, MotionMatix);

% ���ٶȵ����������
ProjAccData = project_accelerate( AccData, RotMatix, MotionMatix);

% �����ٶȵ����
velData     = cal_object_Velocity(ProjAccData, MotionMatix, sampleFreq, 'Time');

% ����λ�Ƶ����
PosData     = cal_object_Position(velData, MotionMatix, sampleFreq, 'Time');

PosData     = zeros(size(PosData));
%������̬����ʾ ������̬�Ƿ�׼ȷ
SamplePlotFreq = 50;
SixDOFanimation1(PosData, RotMatix, ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
                'Position', [9 39 1280 720], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', (sampleFreq / SamplePlotFreq));   