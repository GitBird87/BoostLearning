%导入数据
exp_data     = xlsread('data1.xlsx','base1_data');
base_data    = xlsread('data1.xlsx','base1');

%取出数据
gyr        = exp_data(:,  5:7) / 15; acc      = exp_data(:,  2:4) / 1000;
gyr_base   = base_data(:, 5:7) / 15; acc_base = base_data(:, 2:4) / 1000;
SampleFreq = 100;

%分析基准数据的频谱
fft_analyse(gyr_base(:, 1), SampleFreq, '陀螺仪X');
fft_analyse(gyr_base(:, 2), SampleFreq, '陀螺仪Y');
fft_analyse(gyr_base(:, 3), SampleFreq, '陀螺仪Z');

fft_analyse(acc_base(:, 1), SampleFreq, '加速计X');
fft_analyse(acc_base(:, 2), SampleFreq, '加速计Y');
fft_analyse(acc_base(:, 3), SampleFreq, '加速计Z');

%分析实际数据的频谱
% fft_analyse(gyr(:, 1), SampleFreq, '陀螺仪X');
% fft_analyse(gyr(:, 2), SampleFreq, '陀螺仪Y');
% fft_analyse(gyr(:, 3), SampleFreq, '陀螺仪Z');
% 
% fft_analyse(acc(:, 1), SampleFreq, '加速计X');
% fft_analyse(acc(:, 2), SampleFreq, '加速计Y');
% fft_analyse(acc(:, 3), SampleFreq, '加速计Z');