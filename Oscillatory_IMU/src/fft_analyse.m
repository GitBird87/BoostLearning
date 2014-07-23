function fft_analyse(SignalIn, SampleFreq, TitleName, CutOffFreqH, CutOffFreqL)
% 功  能：对陀螺仪和加速度计的信号进行快速傅里叶变换，以及滤波后进行反变换
% 参  数：(IN/输入参数) ：
%                       SignalIn      输入的信号 向量
%                       SampleFreq    输入信号的频率
%                       TitleName     显示图像的名字
%                       CutOffFreqL   截止频率下限
%                       CutOffFreqH   截止频率上限
%          (OUT/输出参数)：
%                       SignalOut     输出的信号 向量
% 返回值：
% 备  注：1. 频谱的范围是无限的，分析的频谱范围是[0, SampleFreq / 2]
%**************************************************************************

% 设置默认值
switch nargin
    case 1
        SampleFreq  = 100;                                 %设定采样频率为100HZ 
        TitleName   = '输入信号';
        CutOffFreqH = 8;
        CutOffFreqL = 5;
    case 2
        TitleName   = '输入信号';
        CutOffFreqH = 8;
        CutOffFreqL = 5;
    case 3
        CutOffFreqH = 8;
        CutOffFreqL = 5;
    case 4
        CutOffFreqL = 5;
end

SamplePeriod  = 1/SampleFreq;                         %采样时间间隔

SignalLength = length(SignalIn);                      %信号长度
VectorT      = (0 : SignalLength - 1) * SamplePeriod; %时间向量

% 原始图像的绘制
figure; subplot(2, 1, 1);
plot(SampleFreq * VectorT, SignalIn); title([TitleName,'原始信号']); xlabel('时间')

% 原始信号进行傅里叶变换
nfft        = 2^nextpow2(SignalLength);                 %Next power of 2 from length of y
ResultFFT   = fft(SignalIn, nfft) ;                     %对信号y进行FFT变换
halfFFT     = nfft / 2 + 1;
freq        = SampleFreq / 2 * linspace(0,1,halfFFT);   %对频域进行分割

% 绘制频谱的幅度图像
subplot(2, 1, 2);
plot(freq, 2 * abs(ResultFFT(1 : halfFFT)) ) 
title([TitleName,'原始信号的频谱图']);xlabel('Frequency (Hz)'); ylabel('|ResultFFT(freq)|')

%滤波后进行快速傅里叶变换的反变换
CutOffNumH  = ceil(CutOffFreqH / max(freq) * halfFFT);
CutOffNumL  = ceil(CutOffFreqL / max(freq) * halfFFT);
Low_filter  = zeros(length(ResultFFT), 1);
High_filter = zeros(length(ResultFFT), 1);
Mid_filter  = zeros(length(ResultFFT), 1);

%进行滤波操作
Low_filter(1 : CutOffNumL)         = ResultFFT(1 : CutOffNumL);           %滤掉高频成分
Low_filter(nfft-CutOffNumL : nfft) = ResultFFT(nfft-CutOffNumL : nfft);   %滤掉高频成分

High_filter(CutOffNumH : halfFFT)         = ResultFFT(CutOffNumH : halfFFT);          %滤掉低频成分
High_filter(halfFFT : halfFFT+CutOffNumH) = ResultFFT(halfFFT : halfFFT+CutOffNumH);  %滤掉低频成分

Mid_filter(CutOffNumL:CutOffNumH)                 = ResultFFT(CutOffNumL : CutOffNumH);           %滤掉高频成分
Mid_filter(nfft-CutOffNumH:CutOffNumH-CutOffNumL) = ResultFFT(nfft-CutOffNumH:CutOffNumH-CutOffNumL);           %滤掉高频成分

%滤波结果
HighPassRst = real(ifft(High_filter, nfft));
LowPassRst  = real(ifft(Low_filter,  nfft));
MidPassRst  = real(ifft(Mid_filter,  nfft));

figure;subplot(3,1,1);
plot(SampleFreq * VectorT, HighPassRst(1:SignalLength));
title([TitleName, '高通滤波结果']);xlabel('时间');

subplot(3,1,2);
plot(SampleFreq * VectorT, LowPassRst(1:SignalLength));
title([TitleName, '低通滤波结果']);xlabel('时间');

subplot(3,1,3);
plot(SampleFreq * VectorT, MidPassRst(1:SignalLength));
title([TitleName, '带通滤波结果']);xlabel('时间');