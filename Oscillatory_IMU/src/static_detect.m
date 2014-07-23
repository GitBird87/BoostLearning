function [staticMatix, MotionMatix] = static_detect(AccData, initFrame, sampleFreq, thresh, nums)
% ��  �ܣ����ݵ�ǰ�ļ��ٶȷ��ȣ�ȷ��Ŀ��ľ�ֹʱ�̺��˶�ʱ��
% ��  ����(IN/�������) ��
%                       magFlt        ���ٶȵķ���
%                       sampleFreq    ������Ƶ��
%						thresh        ���ٶȵ���С��ֵ
%                       nums          ������ֹ�Ĳ�������
%          (OUT/�������)��
%                       lessMatix     ���������ֵ�ľ�����
% ����ֵ��
% ��  ע��1. �����Ƚϼ򵥣�������Ҫ�޸�
%         2. ����ֵ����������Ҫ���з���
%**************************************************************************

[sizex, sizey] = size(AccData);       %���ݵĳߴ�

if min(sizex, sizey) ~= 3 && min(sizex, sizey) ~= 1
     error('������������');
end

%����ͨ�����д�������ͨ��ֱ�Ӵ���
if min(sizex, sizey) == 3
    if sizex == 3
        AccData = AccData';
    end  
    AccDataMag = AccData(:, 1).*AccData(:, 1) + AccData(:, 2).*AccData(:, 2) + AccData(:, 3).*AccData(:, 3);
    AccDataMag = sqrt(AccDataMag);
else
    AccDataMag = abs(AccData);
end

% ���и�ͨ�˲�
filtCutOff  = 0.001;
[b, a]      = butter(1, 2 * filtCutOff / sampleFreq, 'high');
AccDataMag  = abs(filtfilt(b, a, AccDataMag));

% ���е�ͨ�˲�
filtCutOff  = 6;
[b, a]      = butter(1, 2 * filtCutOff / sampleFreq, 'low');
AccDataMag  = filtfilt(b, a, AccDataMag);

% �趨��ֵ
staticMatix = AccDataMag < thresh;

% ȥ����С�ľ�ֹ����Ͷ�̬����
connum      = 0;
for i = 1 : sizex
   
    if (staticMatix(i) == 1)
        connum = connum + 1;
    else
        if(connum < nums)
            staticMatix(i - connum : i) = 0;
        end
        connum = 0;
    end
    
end

% ��ʼʱ��Ŀ��û���˶�
staticMatix(1 : initFrame) = 1;
MotionMatix                = 1 - staticMatix;
time                       = 1:max(sizex, sizey);
time                       = time / sampleFreq;

figure('Position', [9 39 900 600], 'Number', 'off', 'Name', 'Sensor Data');
subplot(3,1,1);hold on;plot(time, AccData(:, 1), 'r');plot(time, MotionMatix, 'k');title('AccX');
subplot(3,1,2);hold on;plot(time, AccData(:, 2), 'r');plot(time, MotionMatix, 'k');title('AccY');
subplot(3,1,3);hold on;plot(time, AccData(:, 3), 'r');plot(time, MotionMatix, 'k');title('AccZ');

end