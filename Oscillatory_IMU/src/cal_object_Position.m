function PosData = cal_object_Position(VelocityData, MotionMatix, sampleFreq, method)
% ��  �ܣ����ݵ�ǰ����������ٶȼ���Ŀ���λ����Ϣ�����ں�������һЩԼ����������λ�õ�����
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
[sizex, sizey] = size(VelocityData);
if sizex == 3
    VelocityData = VelocityData';
end

PosData = zeros(size(VelocityData));
fmin    = 0.21; fmax  = 15;

% ����λ�ƵĻ������
switch method
    case 'Freq'
        PosData(:, 1) = integral_freq(VelocityData(:, 1), fmin, fmax, sampleFreq, 1);
        PosData(:, 2) = integral_freq(VelocityData(:, 2), fmin, fmax, sampleFreq, 1);
        PosData(:, 3) = integral_freq(VelocityData(:, 3), fmin, fmax, sampleFreq, 1);
    case 'Time'
        PosData(:, 1) = integral_time(VelocityData(:, 1), sampleFreq, 1);
        PosData(:, 2) = integral_time(VelocityData(:, 2), sampleFreq, 1);
        PosData(:, 3) = integral_time(VelocityData(:, 3), sampleFreq, 1);
    otherwise
        error('integral method error...');
end

% ����λ�õ���������

% ����Ļ���
time = 1:max(sizex, sizey);
time = time / sampleFreq;
figure('Number', 'off', 'Name', 'Position');subplot(2,1,1);hold on;
plot(time, PosData(:,1), 'r');plot(time, PosData(:,2), 'g');plot(time, PosData(:,3), 'b');title('position');
legend('X', 'Y', 'Z');

end