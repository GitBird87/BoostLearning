% FFT频谱分析测试的实例程序
clear;clc;close all;

%1. 导入数据
exp_data   = xlsread('data1.xlsx','base1_data');
base_data  = xlsread('data1.xlsx','base1');

%2. 取出数据
gyr        = exp_data(:,  5:7) / 15; acc      = exp_data(:,  2:4) / 1000;
gyr_base   = base_data(:, 5:7) / 15; acc_base = base_data(:, 2:4) / 1000;
SampleFreq = 100;

%3. 分析基准数据的频谱
% fft_analyse(gyr_base(:, 1), SampleFreq, '陀螺仪X', 8, 5);
% fft_analyse(gyr_base(:, 2), SampleFreq, '陀螺仪Y');
% fft_analyse(gyr_base(:, 3), SampleFreq, '陀螺仪Z');
% 
% fft_analyse(acc_base(:, 1), SampleFreq, '加速计X');
% fft_analyse(acc_base(:, 2), SampleFreq, '加速计Y');
% fft_analyse(acc_base(:, 3), SampleFreq, '加速计Z');

%4. 分析实际数据的频谱
fft_analyse(gyr(:, 1), SampleFreq, '陀螺仪X', 20, 0.5);
% fft_analyse(gyr(:, 2), SampleFreq, '陀螺仪Y');
% fft_analyse(gyr(:, 3), SampleFreq, '陀螺仪Z');
% 
% fft_analyse(acc(:, 1), SampleFreq, '加速计X');
% fft_analyse(acc(:, 2), SampleFreq, '加速计Y');
% fft_analyse(acc(:, 3), SampleFreq, '加速计Z');