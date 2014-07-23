%**************************************************************************
% 功  能：进行线性Kalman滤波
% 参  数：(IN/输入参数) ：
%                       InputData     输入的待滤波的数据 一维数组
%						intial        待滤波数据的初始值
%                       DataDev       数据的方差
%                       DataErr
%          (OUT/输出参数)：
%                       KalmanRst     kalman滤波的结果
% 返回值：
% 备  注：方法比较简单，后续需要修改
%**************************************************************************
function KalmanRst = kalman(InputData, intial, DataDev, DataErr)

if nargin == 2
    DataDev = 4e-4; % 过程方差，反应连续两个时刻温度方差。更改查看效果
    DataErr = 0.25; % 测量方差，反应温度计的测量精度。更改查看效果
else if nargin == 3
         DataErr = 0.25; % 测量方差，反应温度计的测量精度。更改查看效果
    end
end

n_iter    = length(InputData); %计算连续n_iter个时刻
arraySize = [n_iter, 1]; % size of array. n_iter行，1列

% 对数组进行初始化
KalmanRst = zeros(arraySize); % 对温度的后验估计。即在k时刻，结合温度计当前测量值与k-1时刻先验估计，得到的最终估计值
P         = zeros(arraySize); % 后验估计的方差
xhatminus = zeros(arraySize); % 温度的先验估计。即在k-1时刻，对k时刻温度做出的估计
Pminus    = zeros(arraySize); % 先验估计的方差
KalGain   = zeros(arraySize); % 卡尔曼增益，反应了温度计测量结果与过程模型（即当前时刻与下一时刻温度相同这一模型）的可信程度

% intial guesses
KalmanRst(1) = intial; %温度初始估计值为23.5度
P(1) =1; %误差方差为1

%DataDev = std(InputData(:));

for k = 2:n_iter
    
    % 时间更新（预测）
    xhatminus(k) = KalmanRst(k-1); %用上一时刻的最优估计值来作为对当前时刻的温度的预测
    Pminus(k)    = P(k-1) + DataDev; %预测的方差为上一时刻温度最优估计值的方差与过程方差之和
    
    % 测量更新（校正）
    KalGain(k)   = Pminus(k) / ( Pminus(k) + DataErr ); %计算卡尔曼增益
    
    %结合当前时刻温度计的测量值，对上一时刻的预测进行校正，得到校正后的最优估计。该估计具有最小均方差
    KalmanRst(k) = xhatminus(k) + KalGain(k) * (InputData(k) - xhatminus(k)); 
    P(k)         = (1 - KalGain(k)) * Pminus(k); %计算最终估计值的方差
end
end
