%**************************************************************************
% 功  能：对陀螺仪和加速度计的信号进行快速傅里叶变换，以及滤波后进行反变换
% 参  数：(IN/输入参数) ：
%                       SignalIn      输入的信号 向量
%          (OUT/输出参数)：
%                       SignalOut     输出的信号 向量
% 返回值：
% 备  注：方法比较简单，后续需要修改
%**************************************************************************
function fft_analyse(SignalIn, SampleFreq, TitleName)

if nargin == 1
    SampleFreq    = 100;                              %设定采样频率为100HZ 
end

SamplePeriod  = 1/SampleFreq;                         %采样时间间隔

SignalLength = length(SignalIn);                      %信号长度
VectorT      = (0 : SignalLength - 1) * SamplePeriod; %时间向量

% 原始图像的绘制
figure; subplot(2, 1, 1);
plot(SampleFreq * VectorT, SignalIn); title([TitleName,'原始信号']); xlabel('时间')

% 原始信号进行傅里叶变换
nfft        = 2^nextpow2(SignalLength);                 %Next power of 2 from length of y
ResultFFT   = fft(SignalIn, nfft) /SignalLength;        %对信号y进行FFT变换
freq        = SampleFreq / 2 * linspace(0,1,nfft/2+1);  %对频域进行分割

% Plot single-sided amplitude spectrum.
subplot(2, 1, 2);
plot(freq, 2 * abs(ResultFFT(1 : nfft / 2 + 1))) 
title([TitleName,'原始信号的频谱图']);xlabel('Frequency (Hz)'); ylabel('|ResultFFT(freq)|')

% %滤波后进行快速傅里叶变换的反变换 以频率15HZ为界，进行滤波
% CutOffFreq = ceil(filter_parameter / max(freq) * (nfft / 2 + 1));
% % 
% Y_filter               = zeros(length(ResultFFT), 1);
% Y_filter(1:CutOffFreq) = ResultFFT(1:CutOffFreq);  %滤掉高频成分
% 
% y_filter = ifft(Y_filter);
% 
% figure
% plot(SampleFreq * VectorT, y_filter(1:SignalLength) * SignalLength * 2)
% title('滤掉高频成分后的信号');xlabel('时间');

% 
% Y_filter=zeros(length(ResultFFT),1);
% Y_filter(M+1:SignalLength,1)=ResultFFT(M+1:SignalLength,1);%滤掉低频成分
% y_filter=ifft(Y_filter);
% % figure
% % plot(SampleFreq*VectorT,y_filter(1:SignalLength)*SignalLength*2)
% % title('滤掉低频成分后的信号')
% % xlabel('时间')
% F_filter=real(y_filter(1:SignalLength)*SignalLength*2); %滤掉高频之后的信号
% 
