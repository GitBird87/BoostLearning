function [EuleAngle, RotMatix] = cal_ery_attitude(AccData, GyrData, SamplePeriod, initFrame, MotionTime)
% ��  �ܣ�����������ݽ��з���������ÿһ��ʱ��Ŀ�����̬
% ��  ����(IN/�������) ��
%                       AccData       ���ٶ�����
%						samplePeriod  ��������
%                       initFrame     ��ʼλ�õ�֡��
%                       MotionTime    ֵΪ1��ʱ��ִ�У�ֵΪ0��ʱ��ִ��
%          (OUT/�������)��
%                       Quaternion    ÿһ�˶�ʱ�̽��ҵ���Ԫ��
% ����ֵ��
% ��  ע������ǰ��Ľ��ҳ�ʼ����̬���ܲ��ǵ�������ϵ�µ���̬�����ǰ����Ҫ�����ж�
%**************************************************************************

FrameNums   = length(MotionTime);       %�����㷨��֡��
Quaternion  = zeros(FrameNums, 4);      %�������̵���Ԫ��
RotMatix    = zeros(3, 3, FrameNums);   %�������̵���ת����
EuleAngle   = zeros(FrameNums, 3);      %�������̵�ŷ����

%�����ʼ���ҵ���̬����ʼʱ����Ϊ����Ϊ��������ϵ��ͨ�����ٶȵĵ�����ת��Ϊ���ҵ�ǰ������ϵ
AHRSalgorithm              = cal_pre_attitude(AccData, SamplePeriod, initFrame);
Quaternion(1:initFrame, :) = repmat(AHRSalgorithm.Quaternion, initFrame, 1);

% ����ÿһ��ʱ�̽��ҵ���̬��λ�� �����е�Ԫ�ؽ�����Ԫ������
for t = initFrame + 1 : FrameNums
    
    % Ŀ�괦�ھ�̬ʱ�̲Ž��������µ�Լ��
    if(MotionTime(t))
        AHRSalgorithm.Kp = 0;
    else
        AHRSalgorithm.Kp = 0.5;
    end
    
    RadSpeed = deg2rad([GyrData(t, 1) GyrData(t, 2) GyrData(t, 3)]);
    
    if (t * SamplePeriod > 35)
        a = 1;
    end
    %������̬�ļ���
    AHRSalgorithm.UpdateIMU(RadSpeed, [AccData(t, 1) AccData(t, 2) AccData(t, 3)]);
    
    Quaternion(t, :)  = AHRSalgorithm.Quaternion;
    
    %ŷ����
    EuleAngle(t, :)   = quatern2euler(AHRSalgorithm.Quaternion) * 180 / pi;
    
    %�������ת�õ�����
    RotMatix(:, :, t) = quatern2rotMat(AHRSalgorithm.Quaternion)';   
   
end

for t = 1 : initFrame
    %ŷ����
    EuleAngle(t, :)   = quatern2euler(Quaternion(t, :)) * 180 / pi;
    
    %�������ת�õ�����
    RotMatix(:, :, t) = quatern2rotMat(Quaternion(t, :))';   
end

%������̬����
time = 1 : FrameNums;
time = time * SamplePeriod;
figure('Number', 'off', 'Name', 'EuleAngle');subplot(2,1,1);hold on;
plot(time, EuleAngle(:,1), 'r');plot(time, EuleAngle(:,2), 'g');plot(time, EuleAngle(:,3), 'b');title('EuleAngle');
legend('X', 'Y', 'Z');

end