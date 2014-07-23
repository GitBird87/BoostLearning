%**************************************************************************
% 功  能：利用加速进行积分
% 参  数：(IN/输入参数) ：
%                       DataIn        输入信号
%						fmin          最小截止频率 (0, sampleFreq)
%                       fmax          最大截止频率 (0, sampleFreq) 
%						sampleFreq    采样频率
%                       it            积分次数
%          (OUT/输出参数)：
%                        dataout      阈值的基准值
% 返回值：无
% 备  注：
%**************************************************************************
function [disint, velint] = integral_acc(acc, sampleFreq)

%一次频域积分计算速度
velint  =  integral_freq(acc, 0.21, 15, 100,1);
velint  =  detrend(velint);

% 计算位移
% disint  =  integral_freq(acc, 0.22, 15,100, 2);
disint  =  integral_freq(velint, 0.2, 8, 100, 1);

% 去除位移中的二次项
sampleFreq = 100;   
t          = 0: 1/sampleFreq : (length(acc)-1)/sampleFreq;
t          = t';
 
figure
plot(t,acc); grid on; %绘制积分前的时程曲线图形
title('加速度');
figure
subplot(2,1,1); plot(t,velint); grid on; %绘制积分前的时程曲线图形
title('频域积分速度');
subplot(2,1,2); plot(t,disint); grid on; %绘制积分后的时程曲线图形
title('频域积分位移');