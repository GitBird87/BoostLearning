%**************************************************************************
% 功  能：对输入的数据进行分析，
% 参  数：(IN/输入参数) ：
%                       AccData       加速度数据
%						samplePeriod  采样周期
%                       initFrame     起始位置的帧数
%          (OUT/输出参数)：
%                       AHRSalgorithm AHRS算法结构体
% 返回值：
% 备  注：由于前面的胶囊初始的姿态可能不是地球坐标系下的姿态，因此前面需要进行判断
%**************************************************************************
function AHRSalgorithm = cal_pre_attitude(AccData, samplePeriod, initFrame)

% 寻找目标初始化状态的时间段 然后在此时间段反复迭代寻找初始时刻的四元数
AHRSalgorithm = AHRS('SamplePeriod', samplePeriod, 'Kp', 1, 'KpInit', 1);
indexSel      = 1 : initFrame;

for i = 1:2000
    AHRSalgorithm.UpdateIMU([0 0 0], [mean(AccData(indexSel, 1)) mean(accY(indexSel, 2)) mean(accZ(indexSel, 3))]);
end


end