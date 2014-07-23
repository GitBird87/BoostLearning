function velData = cal_object_Velocity(ProjAccData, MotionMatix, sampleFreq, method)
% ��  �ܣ����ݵ�ǰ��������ļ��ٶȼ���Ŀ����ٶ���Ϣ�����ں�������һЩԼ�����������ٶȵ�����
% ��  ����(IN/�������) ��
%                       ProjAccData   ͶӰ��ļ��ٶȾ���
%                       MotionMatix   Ŀ���˶���״̬
%                       sampleFreq    ����Ƶ��
%                       method        ���ַ���
%          (OUT/�������)��
%                       velData       ͨ������������ٶ�
% ����ֵ��
% ��  ע��1. �����Ƚϼ򵥣�������Ҫ�޸�
%         2. ����ֵ����������Ҫ���з���
%**************************************************************************

% ��ʼ������
[sizex, sizey] = size(ProjAccData);
if sizex == 3
    ProjAccData = ProjAccData';
end

velData = zeros(size(ProjAccData));
fmin    = 0.21; fmax  = 15;

% �����ٶȵĻ���
switch method
    case 'Freq'
        velData(:, 1) = integral_freq(ProjAccData(:, 1), fmin, fmax, sampleFreq, 1);
        velData(:, 2) = integral_freq(ProjAccData(:, 2), fmin, fmax, sampleFreq, 1);
        velData(:, 3) = integral_freq(ProjAccData(:, 3), fmin, fmax, sampleFreq, 1);
    case 'Time'
        velData(:, 1) = integral_time(ProjAccData(:, 1), sampleFreq, 1);
        velData(:, 2) = integral_time(ProjAccData(:, 2), sampleFreq, 1);
        velData(:, 3) = integral_time(ProjAccData(:, 3), sampleFreq, 1);
    otherwise
        error('integral method error...');
end

% �����ٶȵ���������

% �����ٶ���Ϣ
time = 1:max(sizex, sizey);
time = time / sampleFreq;

figure('Position', [9 39 900 300], 'Number', 'off', 'Name', 'Velocity');
subplot(2,1,1); hold on; 
plot(time, velData(:,1), 'r');plot(time, velData(:,2), 'g');plot(time, velData(:,3), 'b');
title('Velocity');xlabel('Time (s)');ylabel('Velocity (m/s)');legend('X', 'Y', 'Z');

end