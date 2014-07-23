function DataOut = integral_time(DataIn, sampleFreq, flag)
% ��  �ܣ����ò�ͬ�ķ�������ʱ�����
% ��  ����(IN/�������) ��
%                       DataIn        �����ź�
%						sampleFreq    ������Ƶ��
%                       flag          ���ֵķ���
%          (OUT/�������)��
%                       DataOut       ʱ����ֵĽ��
% ����ֵ����
% ��  ע��Ŀǰ�Ļ��ַ����������ӷ� ���η� ����ɭ
%**************************************************************************

%��������ݵĳ�ʼ��
DataOut       = zeros(size(DataIn));
samplePeriod  = 1 / sampleFreq;

switch flag 
    case 1   % ���ӷ�
        for i = 2:length(DataIn)
            DataOut(i) = DataOut(i-1) + DataIn(i) * samplePeriod;
        end
    case 2   % ���η�
        for i = 2:length(DataIn)
            DataOut(i) = DataOut(i-1)+(DataIn(i-1) + DataIn(i))/2 * samplePeriod;
        end
    case 3   % ����ɭ
        DataOut(1) = (DataIn(1)+DataIn(2))*samplePeriod/2;
        for i = 2:length(DataIn)-1
            DataOut(i) = DataOut(i-1) + (DataIn(i-1) + 4 * DataIn(i) + DataIn(i+1)) * samplePeriod / 6;           
        end
        DataOut(length(DataIn)) = DataOut(length(DataIn)-1);
end

end



