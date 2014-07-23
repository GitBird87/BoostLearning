%**************************************************************************
% 功  能：采用不同的方法进行时域积分
% 参  数：(IN/输入参数) ：
%                       DataIn        输入信号
%						FreqTime      周期
%                       flag          积分的方法
%          (OUT/输出参数)：
%                       DataOut      时域积分的结果
% 返回值：无
% 备  注：目前的积分方法包含叠加法 梯形法 辛普森
%**************************************************************************
function DataOut = integral_time(DataIn, FreqTime, flag)

%输出的数据的初始化
DataOut  =  zeros(size(DataIn));

switch flag 
    case 1   % 叠加法
        for i = 2:length(DataIn)
            DataOut(i) = DataOut(i-1) + DataIn(i) * FreqTime;
        end
    case 2   % 梯形法
        for i = 2:length(DataIn)
            DataOut(i) = DataOut(i-1)+(DataIn(i-1) + DataIn(i))/2 * FreqTime;
        end
    case 3   % 辛普森
        DataOut(1) = (DataIn(1)+DataIn(2))*FreqTime/2;
        for i = 2:length(DataIn)-1
            DataOut(i) = DataOut(i-1) + (DataIn(i-1) + 4 * DataIn(i) + DataIn(i+1)) * FreqTime / 6;           
        end
        DataOut(length(DataIn)) = DataOut(length(DataIn)-1);
end

end



