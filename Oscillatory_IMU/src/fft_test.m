% FFTƵ�׷������Ե�ʵ������
clear;clc;close all;

%1. ��������
exp_data   = xlsread('data1.xlsx','base1_data');
base_data  = xlsread('data1.xlsx','base1');

%2. ȡ������
gyr        = exp_data(:,  5:7) / 15; acc      = exp_data(:,  2:4) / 1000;
gyr_base   = base_data(:, 5:7) / 15; acc_base = base_data(:, 2:4) / 1000;
SampleFreq = 100;

%3. ������׼���ݵ�Ƶ��
% fft_analyse(gyr_base(:, 1), SampleFreq, '������X', 8, 5);
% fft_analyse(gyr_base(:, 2), SampleFreq, '������Y');
% fft_analyse(gyr_base(:, 3), SampleFreq, '������Z');
% 
% fft_analyse(acc_base(:, 1), SampleFreq, '���ټ�X');
% fft_analyse(acc_base(:, 2), SampleFreq, '���ټ�Y');
% fft_analyse(acc_base(:, 3), SampleFreq, '���ټ�Z');

%4. ����ʵ�����ݵ�Ƶ��
fft_analyse(gyr(:, 1), SampleFreq, '������X', 20, 0.5);
% fft_analyse(gyr(:, 2), SampleFreq, '������Y');
% fft_analyse(gyr(:, 3), SampleFreq, '������Z');
% 
% fft_analyse(acc(:, 1), SampleFreq, '���ټ�X');
% fft_analyse(acc(:, 2), SampleFreq, '���ټ�Y');
% fft_analyse(acc(:, 3), SampleFreq, '���ټ�Z');