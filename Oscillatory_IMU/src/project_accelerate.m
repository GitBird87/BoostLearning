function ProAccData = project_accelerate( AccData, RotMatix, MotionTime)
% ��  �ܣ��Լ��ٶȼƲ��������ݽ���ͶӰ����������ϵ�£���֤�����ٶȻ��ֵ���Ч��
% ��  ����(IN/�������) ��
%                       AccData       ����kalman�˲���ļ��ٶ�
%						RotMatix      ͨ�����ٶȺ������ǲ������ݼ���õ���Ŀ����̬
%                       MotionTime    Ŀ���˶���ʱ��
%          (OUT/�������)��
%                       ProAccData    ͨ��ͶӰ��������ļ��ٶ�ֵ
% ����ֵ��
% ��  ע��1.����ͶӰ 2.��������
%**************************************************************************

FrameNums  = length(AccData);
ProAccData = zeros(size(AccData));  

% �����������ϵ�µļ��ٶ�
for i = 1:FrameNums
    ProAccData(i,:) = RotMatix(:,:,i) * AccData(i,:)';
end

% ���ڴӾ�ֹ���˶�����ֹ��һ�����ڷ�Χ�� �ٶȴ�0 -> 0 ��˴�ʱ�ļ��ٶȻ���Ӧ��Ϊ0
% �����������ģ����ٶ�Ϊ0���ٶȲ�һ��Ϊ0��
StaticMotion = find_static_motion_time(MotionTime);

% ��ֹʱ���ٶȵ����� �������������Ŀ�꾲ֹ��ˮƽ������ �Խ��Ҳ�����
for i = 1 : FrameNums
    if MotionTime(i) == 0
        ProAccData(i, :) = [0, 0, 1];
    end
end

% ���ü��ٶ�Ϊ0��Ŀ�꾲ֹʱ���ٶ�Ϊ0���Լ��ٶȽ�������
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

%���м��ٶȵĻ���
figure('Number', 'off', 'Name', '''Tilt-Compensated'' accelerometer');
subplot(3,1,1); hold on;plot(ProAccData(:,1), 'r');plot(AccData(:,1), 'g');
subplot(3,1,2); hold on;plot(ProAccData(:,2), 'r');plot(AccData(:,2), 'g');
subplot(3,1,3); hold on;plot(ProAccData(:,3), 'r');plot(AccData(:,3), 'g');

end

function StaticMotion = find_static_motion_time(MotionTime)
% ��  �ܣ������˶��ı�־��ȷ�������˶���ʼ��ʱ��ͽ�����ʱ��
% ��  ����(IN/�������) ��
%                       MotionTime    Ŀ���˶���ʱ��
%          (OUT/�������)��
%                       StaticMotion  ��ֹ�˶��ṹ��
% ����ֵ��
% ��  ע��1.��Ҫ���ǴӾ�ֹ���˶���ʱ�̺ʹ��˶�����ֹʱ�̵Ĳ�һ��
%         2.��Ҫ����Ŀ���˶���ֹʱ�̵ĳ���
%**************************************************************************

MotionEnd          = find([0; diff(MotionTime)] == -1); %��ʼ���뾲ֹʱ��
MotionStart        = find([0; diff(MotionTime)] ==  1); %��ʼ�����˶�ʱ��
StaticMotion.start = [];
StaticMotion.end   = [];

%��ÿ����ʼ�˶�ʱ���ҵ��˶���ֹ��ʱ��
for i = 1 : length(MotionStart)
    for j = 1:length(MotionEnd)
        
        index1 = MotionEnd(j);
        index2 = MotionStart(i);
        
        %��ֹʱ�̴�����ʼʱ�� ���м�Ĺ���һֱ���˶�״̬
        if (index1 > index2) && (sum(MotionTime(index2:index1)) == index1 - index2)
            
            StaticMotion.start = [StaticMotion.start, index2];
            StaticMotion.end   = [StaticMotion.end,   index1];
            break;
            
        end
        
    end
end


end

