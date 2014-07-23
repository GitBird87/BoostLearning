function [EuleAngle, RotMatix] = cal_ery_attitude(AccData, GyrData, SamplePeriod, initFrame, MotionTime)
% 功  能：对输入的数据进行分析，计算每一个时刻目标的姿态
% 参  数：(IN/输入参数) ：
%                       AccData       加速度数据
%						samplePeriod  采样周期
%                       initFrame     起始位置的帧数
%                       MotionTime    值为1的时候执行，值为0的时候不执行
%          (OUT/输出参数)：
%                       Quaternion    每一运动时刻胶囊的四元数
% 返回值：
% 备  注：由于前面的胶囊初始的姿态可能不是地球坐标系下的姿态，因此前面需要进行判断
%**************************************************************************

FrameNums   = length(MotionTime);       %整个算法的帧数
Quaternion  = zeros(FrameNums, 4);      %整个过程的四元素
RotMatix    = zeros(3, 3, FrameNums);   %整个过程的旋转矩阵
EuleAngle   = zeros(FrameNums, 3);      %整个过程的欧拉角

%计算初始胶囊的姿态，开始时刻认为胶囊为地理坐标系，通过加速度的调整，转化为胶囊当前的坐标系
AHRSalgorithm              = cal_pre_attitude(AccData, SamplePeriod, initFrame);
Quaternion(1:initFrame, :) = repmat(AHRSalgorithm.Quaternion, initFrame, 1);

% 计算每一个时刻胶囊的姿态和位置 对所有的元素进行四元数计算
for t = initFrame + 1 : FrameNums
    
    % 目标处于静态时刻才进行重力下的约束
    if(MotionTime(t))
        AHRSalgorithm.Kp = 0;
    else
        AHRSalgorithm.Kp = 0.5;
    end
    
    RadSpeed = deg2rad([GyrData(t, 1) GyrData(t, 2) GyrData(t, 3)]);
    
    if (t * SamplePeriod > 35)
        a = 1;
    end
    %进行姿态的计算
    AHRSalgorithm.UpdateIMU(RadSpeed, [AccData(t, 1) AccData(t, 2) AccData(t, 3)]);
    
    Quaternion(t, :)  = AHRSalgorithm.Quaternion;
    
    %欧拉角
    EuleAngle(t, :)   = quatern2euler(AHRSalgorithm.Quaternion) * 180 / pi;
    
    %这里面的转置的作用
    RotMatix(:, :, t) = quatern2rotMat(AHRSalgorithm.Quaternion)';   
   
end

for t = 1 : initFrame
    %欧拉角
    EuleAngle(t, :)   = quatern2euler(Quaternion(t, :)) * 180 / pi;
    
    %这里面的转置的作用
    RotMatix(:, :, t) = quatern2rotMat(Quaternion(t, :))';   
end

%绘制姿态曲线
time = 1 : FrameNums;
time = time * SamplePeriod;
figure('Number', 'off', 'Name', 'EuleAngle');subplot(2,1,1);hold on;
plot(time, EuleAngle(:,1), 'r');plot(time, EuleAngle(:,2), 'g');plot(time, EuleAngle(:,3), 'b');title('EuleAngle');
legend('X', 'Y', 'Z');

end