function velData = cal_object_Velocity(ProjAccData, MotionMatix, sampleFreq, method)
% 功  能：根据当前的修正后的加速度计算目标的速度信息，并在后续利用一些约束条件进行速度的修正
% 参  数：(IN/输入参数) ：
%                       ProjAccData   投影后的加速度矩阵
%                       MotionMatix   目标运动的状态
%                       sampleFreq    采样频率
%                       method        积分方法
%          (OUT/输出参数)：
%                       velData       通过积分输出的速度
% 返回值：
% 备  注：1. 方法比较简单，后续需要修改
%         2. 在阈值的设置上需要进行分析
%**************************************************************************

% 初始化操作
[sizex, sizey] = size(ProjAccData);
if sizex == 3
    ProjAccData = ProjAccData';
end

velData = zeros(size(ProjAccData));
fmin    = 0.21; fmax  = 15;

% 进行速度的积分
switch method
    case 'Freq'
        velData(:, 1) = integral_freq(ProjAccData(:, 1), fmin, fmax, sampleFreq, 1);
        velData(:, 2) = integral_freq(ProjAccData(:, 2), fmin, fmax, sampleFreq, 1);
        velData(:, 3) = integral_freq(ProjAccData(:, 3), fmin, fmax, sampleFreq, 1);
    case 'Time'
        velData(:, 1) = integral_time(ProjAccData(:, 1), sampleFreq, 1);
        velData(:, 2) = integral_time(ProjAccData(:, 2), sampleFreq, 1);
        velData(:, 3) = integral_time(ProjAccData(:, 3), sampleFreq, 1);
    otherwise
        error('integral method error...');
end

% 进行速度的修正操作

% 绘制速度信息
time = 1:max(sizex, sizey);
time = time / sampleFreq;

figure('Position', [9 39 900 300], 'Number', 'off', 'Name', 'Velocity');
subplot(2,1,1); hold on; 
plot(time, velData(:,1), 'r');plot(time, velData(:,2), 'g');plot(time, velData(:,3), 'b');
title('Velocity');xlabel('Time (s)');ylabel('Velocity (m/s)');legend('X', 'Y', 'Z');

end