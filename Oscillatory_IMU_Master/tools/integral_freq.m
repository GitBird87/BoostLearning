%**************************************************************************
% 功  能：进行频域积分
% 参  数：(IN/输入参数) ：
%                       DataIn        输入信号
%						fmin          最小截止频率 (0, sampleFreq)
%                       fmax          最大截止频率 (0, sampleFreq) 
%						sampleFreq    采样频率
%                       it            积分次数
%          (OUT/输出参数)：
%                        dataout      阈值的基准值
% 返回值：dataout 积分结果
% 备  注：
%**************************************************************************
function dataout =  integral_freq(DataIn, fmin, fmax, sampleFreq, it)

if (fmin > fmax || fmin <= 0 || fmax >= sampleFreq) 
    disp('输入参数错误');
    return;
end

inputData  = DataIn';
dataLength = length(inputData);

%大于并最接近n的2的幂次方为FFT长度
nfft       = 2^nextpow2(dataLength);

%FFT变换
ResultFFT  = fft(inputData, nfft);

%计算频率间隔（Hz/s）
df = sampleFreq / nfft;

%计算指定频带对应频率数组的下标
ni =round(fmin / df + 1);
na =round(fmax / df + 1);

%计算圆频率间隔（rad/s）
dw = 2*pi*df;
%建立正的离散圆频率向量
w1 =  0:dw:2*pi*(0.5*sampleFreq - df);
%建立负的离散圆频率向量
w2 = -2*pi*(0.5*sampleFreq - df):dw:0;
%将正负圆频率向量组合成一个向量
w  = 1i *[w1,w2];
%以积分次数为指数，建立圆频率变量向量
w  = w.^it;

%进行积分的频域变换
b=zeros(1,nfft); b(2:nfft-1) =ResultFFT(2:nfft-1)./w(2:nfft-1);
a=zeros(1,nfft);

%消除指定正负频带外的频率成分
a(ni:na)               = b(ni:na);
a(nfft-na+1:nfft-ni+1) = b(nfft-na+1:nfft-ni+1);

%IFFT变换
ResultIFFT = ifft(a, nfft); 
%取逆变换的实部n个元素并乘以单位变换系数为积分结果
dataout    = real(ResultIFFT(1:dataLength));
dataout    = dataout';
end