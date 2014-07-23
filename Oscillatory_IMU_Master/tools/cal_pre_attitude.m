%**************************************************************************
% ��  �ܣ�����������ݽ��з�����
% ��  ����(IN/�������) ��
%                       AccData       ���ٶ�����
%						samplePeriod  ��������
%                       initFrame     ��ʼλ�õ�֡��
%          (OUT/�������)��
%                       AHRSalgorithm AHRS�㷨�ṹ��
% ����ֵ��
% ��  ע������ǰ��Ľ��ҳ�ʼ����̬���ܲ��ǵ�������ϵ�µ���̬�����ǰ����Ҫ�����ж�
%**************************************************************************
function AHRSalgorithm = cal_pre_attitude(AccData, samplePeriod, initFrame)

% Ѱ��Ŀ���ʼ��״̬��ʱ��� Ȼ���ڴ�ʱ��η�������Ѱ�ҳ�ʼʱ�̵���Ԫ��
AHRSalgorithm = AHRS('SamplePeriod', samplePeriod, 'Kp', 1, 'KpInit', 1);
indexSel      = 1 : initFrame;

for i = 1:2000
    AHRSalgorithm.UpdateIMU([0 0 0], [mean(AccData(indexSel, 1)) mean(accY(indexSel, 2)) mean(accZ(indexSel, 3))]);
end


end