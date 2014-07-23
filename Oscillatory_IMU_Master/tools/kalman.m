%**************************************************************************
% ��  �ܣ���������Kalman�˲�
% ��  ����(IN/�������) ��
%                       InputData     ����Ĵ��˲������� һά����
%						intial        ���˲����ݵĳ�ʼֵ
%                       DataDev       ���ݵķ���
%                       DataErr
%          (OUT/�������)��
%                       KalmanRst     kalman�˲��Ľ��
% ����ֵ��
% ��  ע�������Ƚϼ򵥣�������Ҫ�޸�
%**************************************************************************
function KalmanRst = kalman(InputData, intial, DataDev, DataErr)

if nargin == 2
    DataDev = 4e-4; % ���̷����Ӧ��������ʱ���¶ȷ�����Ĳ鿴Ч��
    DataErr = 0.25; % ���������Ӧ�¶ȼƵĲ������ȡ����Ĳ鿴Ч��
else if nargin == 3
         DataErr = 0.25; % ���������Ӧ�¶ȼƵĲ������ȡ����Ĳ鿴Ч��
    end
end

n_iter    = length(InputData); %��������n_iter��ʱ��
arraySize = [n_iter, 1]; % size of array. n_iter�У�1��

% ��������г�ʼ��
KalmanRst = zeros(arraySize); % ���¶ȵĺ�����ơ�����kʱ�̣�����¶ȼƵ�ǰ����ֵ��k-1ʱ��������ƣ��õ������չ���ֵ
P         = zeros(arraySize); % ������Ƶķ���
xhatminus = zeros(arraySize); % �¶ȵ�������ơ�����k-1ʱ�̣���kʱ���¶������Ĺ���
Pminus    = zeros(arraySize); % ������Ƶķ���
KalGain   = zeros(arraySize); % ���������棬��Ӧ���¶ȼƲ�����������ģ�ͣ�����ǰʱ������һʱ���¶���ͬ��һģ�ͣ��Ŀ��ų̶�

% intial guesses
KalmanRst(1) = intial; %�¶ȳ�ʼ����ֵΪ23.5��
P(1) =1; %����Ϊ1

%DataDev = std(InputData(:));

for k = 2:n_iter
    
    % ʱ����£�Ԥ�⣩
    xhatminus(k) = KalmanRst(k-1); %����һʱ�̵����Ź���ֵ����Ϊ�Ե�ǰʱ�̵��¶ȵ�Ԥ��
    Pminus(k)    = P(k-1) + DataDev; %Ԥ��ķ���Ϊ��һʱ���¶����Ź���ֵ�ķ�������̷���֮��
    
    % �������£�У����
    KalGain(k)   = Pminus(k) / ( Pminus(k) + DataErr ); %���㿨��������
    
    %��ϵ�ǰʱ���¶ȼƵĲ���ֵ������һʱ�̵�Ԥ�����У�����õ�У��������Ź��ơ��ù��ƾ�����С������
    KalmanRst(k) = xhatminus(k) + KalGain(k) * (InputData(k) - xhatminus(k)); 
    P(k)         = (1 - KalGain(k)) * Pminus(k); %�������չ���ֵ�ķ���
end
end
