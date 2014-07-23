%**************************************************************************
% ��  �ܣ����ü��ٽ��л���
% ��  ����(IN/�������) ��
%                       DataIn        �����ź�
%						fmin          ��С��ֹƵ�� (0, sampleFreq)
%                       fmax          ����ֹƵ�� (0, sampleFreq) 
%						sampleFreq    ����Ƶ��
%                       it            ���ִ���
%          (OUT/�������)��
%                        dataout      ��ֵ�Ļ�׼ֵ
% ����ֵ����
% ��  ע��
%**************************************************************************
function [disint, velint] = integral_acc(acc, sampleFreq)

%һ��Ƶ����ּ����ٶ�
velint  =  integral_freq(acc, 0.21, 15, 100,1);
velint  =  detrend(velint);

% ����λ��
% disint  =  integral_freq(acc, 0.22, 15,100, 2);
disint  =  integral_freq(velint, 0.2, 8, 100, 1);

% ȥ��λ���еĶ�����
sampleFreq = 100;   
t          = 0: 1/sampleFreq : (length(acc)-1)/sampleFreq;
t          = t';
 
figure
plot(t,acc); grid on; %���ƻ���ǰ��ʱ������ͼ��
title('���ٶ�');
figure
subplot(2,1,1); plot(t,velint); grid on; %���ƻ���ǰ��ʱ������ͼ��
title('Ƶ������ٶ�');
subplot(2,1,2); plot(t,disint); grid on; %���ƻ��ֺ��ʱ������ͼ��
title('Ƶ�����λ��');