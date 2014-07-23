%��������
exp_data     = xlsread('data1.xlsx','base1_data');
base_data    = xlsread('data1.xlsx','base1');

%ȡ������
gyr        = exp_data(:,  5:7) / 15; acc      = exp_data(:,  2:4) / 1000;
gyr_base   = base_data(:, 5:7) / 15; acc_base = base_data(:, 2:4) / 1000;
SampleFreq = 100;

%������׼���ݵ�Ƶ��
fft_analyse(gyr_base(:, 1), SampleFreq, '������X');
fft_analyse(gyr_base(:, 2), SampleFreq, '������Y');
fft_analyse(gyr_base(:, 3), SampleFreq, '������Z');

fft_analyse(acc_base(:, 1), SampleFreq, '���ټ�X');
fft_analyse(acc_base(:, 2), SampleFreq, '���ټ�Y');
fft_analyse(acc_base(:, 3), SampleFreq, '���ټ�Z');

%����ʵ�����ݵ�Ƶ��
% fft_analyse(gyr(:, 1), SampleFreq, '������X');
% fft_analyse(gyr(:, 2), SampleFreq, '������Y');
% fft_analyse(gyr(:, 3), SampleFreq, '������Z');
% 
% fft_analyse(acc(:, 1), SampleFreq, '���ټ�X');
% fft_analyse(acc(:, 2), SampleFreq, '���ټ�Y');
% fft_analyse(acc(:, 3), SampleFreq, '���ټ�Z');