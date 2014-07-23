function fft_analyse(SignalIn, SampleFreq, TitleName, CutOffFreqH, CutOffFreqL)
% ��  �ܣ��������Ǻͼ��ٶȼƵ��źŽ��п��ٸ���Ҷ�任���Լ��˲�����з��任
% ��  ����(IN/�������) ��
%                       SignalIn      ������ź� ����
%                       SampleFreq    �����źŵ�Ƶ��
%                       TitleName     ��ʾͼ�������
%                       CutOffFreqL   ��ֹƵ������
%                       CutOffFreqH   ��ֹƵ������
%          (OUT/�������)��
%                       SignalOut     ������ź� ����
% ����ֵ��
% ��  ע��1. Ƶ�׵ķ�Χ�����޵ģ�������Ƶ�׷�Χ��[0, SampleFreq / 2]
%**************************************************************************

% ����Ĭ��ֵ
switch nargin
    case 1
        SampleFreq  = 100;                                 %�趨����Ƶ��Ϊ100HZ 
        TitleName   = '�����ź�';
        CutOffFreqH = 8;
        CutOffFreqL = 5;
    case 2
        TitleName   = '�����ź�';
        CutOffFreqH = 8;
        CutOffFreqL = 5;
    case 3
        CutOffFreqH = 8;
        CutOffFreqL = 5;
    case 4
        CutOffFreqL = 5;
end

SamplePeriod  = 1/SampleFreq;                         %����ʱ����

SignalLength = length(SignalIn);                      %�źų���
VectorT      = (0 : SignalLength - 1) * SamplePeriod; %ʱ������

% ԭʼͼ��Ļ���
figure; subplot(2, 1, 1);
plot(SampleFreq * VectorT, SignalIn); title([TitleName,'ԭʼ�ź�']); xlabel('ʱ��')

% ԭʼ�źŽ��и���Ҷ�任
nfft        = 2^nextpow2(SignalLength);                 %Next power of 2 from length of y
ResultFFT   = fft(SignalIn, nfft) ;                     %���ź�y����FFT�任
halfFFT     = nfft / 2 + 1;
freq        = SampleFreq / 2 * linspace(0,1,halfFFT);   %��Ƶ����зָ�

% ����Ƶ�׵ķ���ͼ��
subplot(2, 1, 2);
plot(freq, 2 * abs(ResultFFT(1 : halfFFT)) ) 
title([TitleName,'ԭʼ�źŵ�Ƶ��ͼ']);xlabel('Frequency (Hz)'); ylabel('|ResultFFT(freq)|')

%�˲�����п��ٸ���Ҷ�任�ķ��任
CutOffNumH  = ceil(CutOffFreqH / max(freq) * halfFFT);
CutOffNumL  = ceil(CutOffFreqL / max(freq) * halfFFT);
Low_filter  = zeros(length(ResultFFT), 1);
High_filter = zeros(length(ResultFFT), 1);
Mid_filter  = zeros(length(ResultFFT), 1);

%�����˲�����
Low_filter(1 : CutOffNumL)         = ResultFFT(1 : CutOffNumL);           %�˵���Ƶ�ɷ�
Low_filter(nfft-CutOffNumL : nfft) = ResultFFT(nfft-CutOffNumL : nfft);   %�˵���Ƶ�ɷ�

High_filter(CutOffNumH : halfFFT)         = ResultFFT(CutOffNumH : halfFFT);          %�˵���Ƶ�ɷ�
High_filter(halfFFT : halfFFT+CutOffNumH) = ResultFFT(halfFFT : halfFFT+CutOffNumH);  %�˵���Ƶ�ɷ�

Mid_filter(CutOffNumL:CutOffNumH)                 = ResultFFT(CutOffNumL : CutOffNumH);           %�˵���Ƶ�ɷ�
Mid_filter(nfft-CutOffNumH:CutOffNumH-CutOffNumL) = ResultFFT(nfft-CutOffNumH:CutOffNumH-CutOffNumL);           %�˵���Ƶ�ɷ�

%�˲����
HighPassRst = real(ifft(High_filter, nfft));
LowPassRst  = real(ifft(Low_filter,  nfft));
MidPassRst  = real(ifft(Mid_filter,  nfft));

figure;subplot(3,1,1);
plot(SampleFreq * VectorT, HighPassRst(1:SignalLength));
title([TitleName, '��ͨ�˲����']);xlabel('ʱ��');

subplot(3,1,2);
plot(SampleFreq * VectorT, LowPassRst(1:SignalLength));
title([TitleName, '��ͨ�˲����']);xlabel('ʱ��');

subplot(3,1,3);
plot(SampleFreq * VectorT, MidPassRst(1:SignalLength));
title([TitleName, '��ͨ�˲����']);xlabel('ʱ��');