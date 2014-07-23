%���������źŽ��п��ٸ���Ҷ�任���Լ��˲�����з��任

function F_filter=signalfft(signal, filter_parameter, run_mode, titles)

%��������ض���ϵͳ
Fs =100;              %�趨����Ƶ��Ϊ100HZ
T  = 1/Fs;            %����ʱ����
L  = length(signal);  %�źų���
t  = (0:L-1)*T;       %ʱ������
y  = signal;

figure;
subplot(3, 1, 1); plot(Fs*t,y);
title(['ԭʼ�ź�',titles]); xlabel('ʱ��')

NFFT = 2^nextpow2(L);                          %Next power of 2 from length of y
Y    = fft(y, NFFT) / L;                       %���ź�y����FFT�任
f    = Fs / 2 * linspace(0, 1, NFFT / 2 + 1);  %��Ƶ����зָ�

subplot(3, 1, 2);plot(f, 2 * abs(Y(1:NFFT/2+1))) 
title('ԭʼ�źŵ�Ƶ��ͼ'); xlabel('Frequency (Hz)'); ylabel('|Y(f)|')

%�˲�����п��ٸ���Ҷ�任�ķ��任
%��Ƶ��15HZΪ�磬�����˲�
M=ceil(filter_parameter/max(f)*(NFFT/2+1));

Y_filter = zeros(length(Y),1);

%���е�ͨ�˲�
if run_mode == 0
    Y_filter(1:M,1) = Y(1:M,1); 
    y_filter        = ifft(Y_filter);
%���и�ͨ�˲�
else 
    Y_filter(M+1:L,1)=Y(M+1:L,1);
    y_filter=ifft(Y_filter);
end

F_filter=y_filter(1:L)*L*2; %�˵���Ƶ֮����ź�

subplot(3, 1, 3);plot(Fs*t,y_filter(1:L)*L*2)
title('�˵���Ľ��')
xlabel('ʱ��')
