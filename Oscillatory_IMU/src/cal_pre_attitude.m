function AHRSalgorithm = cal_pre_attitude(AccData, samplePeriod, initFrame)
% 功  能：对输入的数据进行分析，计算目标初始时刻的姿态
% 参  数：(IN/输入参数) ：
%                       AccData       加速度数据
%						samplePeriod  采样周期
%                       initFrame     起始位置的帧数
%          (OUT/输出参数)：
%                       AHRSalgorithm AHRS算法结构体
% 返回值：
% 备  注：由于前面的胶囊初始的姿态可能不是地球坐标系下的姿态，因此前面需要进行判断
%**************************************************************************

addpath('AHRS')

% 寻找目标初始化状态的时间段 然后在此时间段反复迭代寻找初始时刻的四元数
% 有两种计算的方法，这里首先采用第一种
AHRSalgorithm  = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1, 'Ki', 1); 
%AHRSalgorithm = MadgwickAHRS('samplePeriod', samplePeriod, 'Beta', 0.1);

indexSel       = 1 : initFrame;

%初始时刻的均值
mean_x = mean(AccData(indexSel, 1));
mean_y = mean(AccData(indexSel, 2));
mean_z = mean(AccData(indexSel, 3));

%进行初始姿态的求解
for i = 1:2000
    AHRSalgorithm.UpdateIMU([0 0 0], [mean_x, mean_y, mean_z]);
end


end