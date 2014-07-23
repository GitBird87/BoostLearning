%**************************************************************************
% ��  �ܣ��������Ǻͼ��ٶȼƵ��źŽ��п��ٸ���Ҷ�任���Լ��˲�����з��任
% ��  ����(IN/�������) ��
%                       SignalIn      ������ź� ����
%          (OUT/�������)��
%                       SignalOut     ������ź� ����
% ����ֵ��
% ��  ע�������Ƚϼ򵥣�������Ҫ�޸�
%**************************************************************************
function fft_analyse(SignalIn, SampleFreq, TitleName)

if nargin == 1
    SampleFreq    = 100;                              %�趨����Ƶ��Ϊ100HZ 
end

SamplePeriod  = 1/SampleFreq;                         %����ʱ����

SignalLength = length(SignalIn);                      %�źų���
VectorT      = (0 : SignalLength - 1) * SamplePeriod; %ʱ������

% ԭʼͼ��Ļ���
figure; subplot(2, 1, 1);
plot(SampleFreq * VectorT, SignalIn); title([TitleName,'ԭʼ�ź�']); xlabel('ʱ��')

% ԭʼ�źŽ��и���Ҷ�任
nfft        = 2^nextpow2(SignalLength);                 %Next power of 2 from length of y
ResultFFT   = fft(SignalIn, nfft) /SignalLength;        %���ź�y����FFT�任
freq        = SampleFreq / 2 * linspace(0,1,nfft/2+1);  %��Ƶ����зָ�

% Plot single-sided amplitude spectrum.
subplot(2, 1, 2);
plot(freq, 2 * abs(ResultFFT(1 : nfft / 2 + 1))) 
title([TitleName,'ԭʼ�źŵ�Ƶ��ͼ']);xlabel('Frequency (Hz)'); ylabel('|ResultFFT(freq)|')

% %�˲�����п��ٸ���Ҷ�任�ķ��任 ��Ƶ��15HZΪ�磬�����˲�
% CutOffFreq = ceil(filter_parameter / max(freq) * (nfft / 2 + 1));
% % 
% Y_filter               = zeros(length(ResultFFT), 1);
% Y_filter(1:CutOffFreq) = ResultFFT(1:CutOffFreq);  %�˵���Ƶ�ɷ�
% 
% y_filter = ifft(Y_filter);
% 
% figure
% plot(SampleFreq * VectorT, y_filter(1:SignalLength) * SignalLength * 2)
% title('�˵���Ƶ�ɷֺ���ź�');xlabel('ʱ��');

% 
% Y_filter=zeros(length(ResultFFT),1);
% Y_filter(M+1:SignalLength,1)=ResultFFT(M+1:SignalLength,1);%�˵���Ƶ�ɷ�
% y_filter=ifft(Y_filter);
% % figure
% % plot(SampleFreq*VectorT,y_filter(1:SignalLength)*SignalLength*2)
% % title('�˵���Ƶ�ɷֺ���ź�')
% % xlabel('ʱ��')
% F_filter=real(y_filter(1:SignalLength)*SignalLength*2); %�˵���Ƶ֮����ź�
% 
