%**************************************************************************
% ��  �ܣ�����Ƶ�����
% ��  ����(IN/�������) ��
%                       DataIn        �����ź�
%						fmin          ��С��ֹƵ�� (0, sampleFreq)
%                       fmax          ����ֹƵ�� (0, sampleFreq) 
%						sampleFreq    ����Ƶ��
%                       it            ���ִ���
%          (OUT/�������)��
%                        dataout      ��ֵ�Ļ�׼ֵ
% ����ֵ��dataout ���ֽ��
% ��  ע��
%**************************************************************************
function dataout =  integral_freq(DataIn, fmin, fmax, sampleFreq, it)

if (fmin > fmax || fmin <= 0 || fmax >= sampleFreq) 
    disp('�����������');
    return;
end

inputData  = DataIn';
dataLength = length(inputData);

%���ڲ���ӽ�n��2���ݴη�ΪFFT����
nfft       = 2^nextpow2(dataLength);

%FFT�任
ResultFFT  = fft(inputData, nfft);

%����Ƶ�ʼ����Hz/s��
df = sampleFreq / nfft;

%����ָ��Ƶ����ӦƵ��������±�
ni =round(fmin / df + 1);
na =round(fmax / df + 1);

%����ԲƵ�ʼ����rad/s��
dw = 2*pi*df;
%����������ɢԲƵ������
w1 =  0:dw:2*pi*(0.5*sampleFreq - df);
%����������ɢԲƵ������
w2 = -2*pi*(0.5*sampleFreq - df):dw:0;
%������ԲƵ��������ϳ�һ������
w  = 1i *[w1,w2];
%�Ի��ִ���Ϊָ��������ԲƵ�ʱ�������
w  = w.^it;

%���л��ֵ�Ƶ��任
b=zeros(1,nfft); b(2:nfft-1) =ResultFFT(2:nfft-1)./w(2:nfft-1);
a=zeros(1,nfft);

%����ָ������Ƶ�����Ƶ�ʳɷ�
a(ni:na)               = b(ni:na);
a(nfft-na+1:nfft-ni+1) = b(nfft-na+1:nfft-ni+1);

%IFFT�任
ResultIFFT = ifft(a, nfft); 
%ȡ��任��ʵ��n��Ԫ�ز����Ե�λ�任ϵ��Ϊ���ֽ��
dataout    = real(ResultIFFT(1:dataLength));
dataout    = dataout';
end