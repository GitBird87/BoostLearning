function AHRSalgorithm = cal_pre_attitude(AccData, samplePeriod, initFrame)
% ��  �ܣ�����������ݽ��з���������Ŀ���ʼʱ�̵���̬
% ��  ����(IN/�������) ��
%                       AccData       ���ٶ�����
%						samplePeriod  ��������
%                       initFrame     ��ʼλ�õ�֡��
%          (OUT/�������)��
%                       AHRSalgorithm AHRS�㷨�ṹ��
% ����ֵ��
% ��  ע������ǰ��Ľ��ҳ�ʼ����̬���ܲ��ǵ�������ϵ�µ���̬�����ǰ����Ҫ�����ж�
%**************************************************************************

addpath('AHRS')

% Ѱ��Ŀ���ʼ��״̬��ʱ��� Ȼ���ڴ�ʱ��η�������Ѱ�ҳ�ʼʱ�̵���Ԫ��
% �����ּ���ķ������������Ȳ��õ�һ��
AHRSalgorithm  = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1, 'Ki', 1); 
%AHRSalgorithm = MadgwickAHRS('samplePeriod', samplePeriod, 'Beta', 0.1);

indexSel       = 1 : initFrame;

%��ʼʱ�̵ľ�ֵ
mean_x = mean(AccData(indexSel, 1));
mean_y = mean(AccData(indexSel, 2));
mean_z = mean(AccData(indexSel, 3));

%���г�ʼ��̬�����
for i = 1:2000
    AHRSalgorithm.UpdateIMU([0 0 0], [mean_x, mean_y, mean_z]);
end


end