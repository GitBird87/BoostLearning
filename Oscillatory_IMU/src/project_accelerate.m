function ProAccData = project_accelerate( AccData, RotMatix, MotionTime)
% 功  能：对加速度计测量的数据进行投影到地球坐标系下，保证后续速度积分的有效性
% 参  数：(IN/输入参数) ：
%                       AccData       经过kalman滤波后的加速度
%						RotMatix      通过加速度和陀螺仪测量数据计算得到的目标姿态
%                       MotionTime    目标运动的时间
%          (OUT/输出参数)：
%                       ProAccData    通过投影和修正后的加速度值
% 返回值：
% 备  注：1.进行投影 2.进行修正
%**************************************************************************

FrameNums  = length(AccData);
ProAccData = zeros(size(AccData));  

% 计算地球坐标系下的加速度
for i = 1:FrameNums
    ProAccData(i,:) = RotMatix(:,:,i) * AccData(i,:)';
end

% 由于从静止到运动到静止的一个周期范围内 速度从0 -> 0 因此此时的加速度积分应该为0
% 这里存在问题的，加速度为0，速度不一定为0的
StaticMotion = find_static_motion_time(MotionTime);

% 静止时刻速度的修正 这里仅仅局限于目标静止在水平的桌面 对胶囊不适用
for i = 1 : FrameNums
    if MotionTime(i) == 0
        ProAccData(i, :) = [0, 0, 1];
    end
end

% 利用加速度为0后，目标静止时刻速度为0来对加速度进行修正
for i = 1 : length(StaticMotion.start)
    idx1 = StaticMotion.start(i);
    idx2 = StaticMotion.end(i);
    for j = 1:2
        meanValue = mean(ProAccData(idx1:idx2, j));
        ProAccData(idx1:idx2, j) = ProAccData(idx1:idx2, j) - meanValue;
    end
    
    meanValue = mean(ProAccData(idx1:idx2, 3)) - 1;
    ProAccData(idx1:idx2, 3) = ProAccData(idx1:idx2, 3) - meanValue;
end

ProAccData(:, 3) = ProAccData(:, 3) - 1;

%进行加速度的绘制
figure('Number', 'off', 'Name', '''Tilt-Compensated'' accelerometer');
subplot(3,1,1); hold on;plot(ProAccData(:,1), 'r');plot(AccData(:,1), 'g');
subplot(3,1,2); hold on;plot(ProAccData(:,2), 'r');plot(AccData(:,2), 'g');
subplot(3,1,3); hold on;plot(ProAccData(:,3), 'r');plot(AccData(:,3), 'g');

end

function StaticMotion = find_static_motion_time(MotionTime)
% 功  能：根据运动的标志来确定周期运动开始的时间和结束的时间
% 参  数：(IN/输入参数) ：
%                       MotionTime    目标运动的时间
%          (OUT/输出参数)：
%                       StaticMotion  静止运动结构体
% 返回值：
% 备  注：1.需要考虑从静止到运动的时刻和从运动到静止时刻的不一致
%         2.需要考虑目标运动静止时刻的长度
%**************************************************************************

MotionEnd          = find([0; diff(MotionTime)] == -1); %开始进入静止时刻
MotionStart        = find([0; diff(MotionTime)] ==  1); %开始进行运动时刻
StaticMotion.start = [];
StaticMotion.end   = [];

%对每个开始运动时刻找到运动静止的时刻
for i = 1 : length(MotionStart)
    for j = 1:length(MotionEnd)
        
        index1 = MotionEnd(j);
        index2 = MotionStart(i);
        
        %终止时刻大于起始时刻 且中间的过程一直是运动状态
        if (index1 > index2) && (sum(MotionTime(index2:index1)) == index1 - index2)
            
            StaticMotion.start = [StaticMotion.start, index2];
            StaticMotion.end   = [StaticMotion.end,   index1];
            break;
            
        end
        
    end
end


end

