%对陀螺仪信号进行快速傅里叶变换，以及滤波后进行反变换

function F_filter=signalfft(signal, filter_parameter, run_mode, titles)

%针对我们特定的系统
Fs =100;              %设定采样频率为100HZ
T  = 1/Fs;            %采样时间间隔
L  = length(signal);  %信号长度
t  = (0:L-1)*T;       %时间向量
y  = signal;

figure;
subplot(3, 1, 1); plot(Fs*t,y);
title(['原始信号',titles]); xlabel('时间')

NFFT = 2^nextpow2(L);                          %Next power of 2 from length of y
Y    = fft(y, NFFT) / L;                       %对信号y进行FFT变换
f    = Fs / 2 * linspace(0, 1, NFFT / 2 + 1);  %对频域进行分割

subplot(3, 1, 2);plot(f, 2 * abs(Y(1:NFFT/2+1))) 
title('原始信号的频谱图'); xlabel('Frequency (Hz)'); ylabel('|Y(f)|')

%滤波后进行快速傅里叶变换的反变换
%以频率15HZ为界，进行滤波
M=ceil(filter_parameter/max(f)*(NFFT/2+1));

Y_filter = zeros(length(Y),1);

%进行低通滤波
if run_mode == 0
    Y_filter(1:M,1) = Y(1:M,1); 
    y_filter        = ifft(Y_filter);
%进行高通滤波
else 
    Y_filter(M+1:L,1)=Y(M+1:L,1);
    y_filter=ifft(Y_filter);
end

F_filter=y_filter(1:L)*L*2; %滤掉高频之后的信号

subplot(3, 1, 3);plot(Fs*t,y_filter(1:L)*L*2)
title('滤掉后的结果')
xlabel('时间')
