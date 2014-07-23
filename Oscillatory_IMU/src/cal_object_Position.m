function PosData = cal_object_Position(VelocityData, MotionMatix, sampleFreq, method)
% 功  能：根据当前的修正后的速度计算目标的位置信息，并在后续利用一些约束条件进行位置的修正
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
[sizex, sizey] = size(VelocityData);
if sizex == 3
    VelocityData = VelocityData';
end

PosData = zeros(size(VelocityData));
fmin    = 0.21; fmax  = 15;

% 进行位移的积分求解
switch method
    case 'Freq'
        PosData(:, 1) = integral_freq(VelocityData(:, 1), fmin, fmax, sampleFreq, 1);
        PosData(:, 2) = integral_freq(VelocityData(:, 2), fmin, fmax, sampleFreq, 1);
        PosData(:, 3) = integral_freq(VelocityData(:, 3), fmin, fmax, sampleFreq, 1);
    case 'Time'
        PosData(:, 1) = integral_time(VelocityData(:, 1), sampleFreq, 1);
        PosData(:, 2) = integral_time(VelocityData(:, 2), sampleFreq, 1);
        PosData(:, 3) = integral_time(VelocityData(:, 3), sampleFreq, 1);
    otherwise
        error('integral method error...');
end

% 进行位置的修正操作

% 结果的绘制
time = 1:max(sizex, sizey);
time = time / sampleFreq;
figure('Number', 'off', 'Name', 'Position');subplot(2,1,1);hold on;
plot(time, PosData(:,1), 'r');plot(time, PosData(:,2), 'g');plot(time, PosData(:,3), 'b');title('position');
legend('X', 'Y', 'Z');

end