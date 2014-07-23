function [staticMatix, MotionMatix] = static_detect(AccData, initFrame, sampleFreq, thresh, nums)
% 功  能：根据当前的加速度幅度，确定目标的静止时刻和运动时刻
% 参  数：(IN/输入参数) ：
%                       magFlt        加速度的幅度
%                       sampleFreq    采样的频率
%						thresh        加速度的最小阈值
%                       nums          连续静止的采样次数
%          (OUT/输出参数)：
%                       lessMatix     输出低于阈值的矩阵数
% 返回值：
% 备  注：1. 方法比较简单，后续需要修改
%         2. 在阈值的设置上需要进行分析
%**************************************************************************

[sizex, sizey] = size(AccData);       %数据的尺寸

if min(sizex, sizey) ~= 3 && min(sizex, sizey) ~= 1
     error('参数出入有误');
end

%三个通道集中处理，单个通道直接处理
if min(sizex, sizey) == 3
    if sizex == 3
        AccData = AccData';
    end  
    AccDataMag = AccData(:, 1).*AccData(:, 1) + AccData(:, 2).*AccData(:, 2) + AccData(:, 3).*AccData(:, 3);
    AccDataMag = sqrt(AccDataMag);
else
    AccDataMag = abs(AccData);
end

% 进行高通滤波
filtCutOff  = 0.001;
[b, a]      = butter(1, 2 * filtCutOff / sampleFreq, 'high');
AccDataMag  = abs(filtfilt(b, a, AccDataMag));

% 进行低通滤波
filtCutOff  = 6;
[b, a]      = butter(1, 2 * filtCutOff / sampleFreq, 'low');
AccDataMag  = filtfilt(b, a, AccDataMag);

% 设定阈值
staticMatix = AccDataMag < thresh;

% 去除过小的静止区域和动态区域
connum      = 0;
for i = 1 : sizex
   
    if (staticMatix(i) == 1)
        connum = connum + 1;
    else
        if(connum < nums)
            staticMatix(i - connum : i) = 0;
        end
        connum = 0;
    end
    
end

% 初始时刻目标没有运动
staticMatix(1 : initFrame) = 1;
MotionMatix                = 1 - staticMatix;
time                       = 1:max(sizex, sizey);
time                       = time / sampleFreq;

figure('Position', [9 39 900 600], 'Number', 'off', 'Name', 'Sensor Data');
subplot(3,1,1);hold on;plot(time, AccData(:, 1), 'r');plot(time, MotionMatix, 'k');title('AccX');
subplot(3,1,2);hold on;plot(time, AccData(:, 2), 'r');plot(time, MotionMatix, 'k');title('AccY');
subplot(3,1,3);hold on;plot(time, AccData(:, 3), 'r');plot(time, MotionMatix, 'k');title('AccZ');

end