%**************************************************************************
% ��  �ܣ����ò�ͬ�ķ�������ʱ�����
% ��  ����(IN/�������) ��
%                       DataIn        �����ź�
%						FreqTime      ����
%                       flag          ���ֵķ���
%          (OUT/�������)��
%                       DataOut      ʱ����ֵĽ��
% ����ֵ����
% ��  ע��Ŀǰ�Ļ��ַ����������ӷ� ���η� ����ɭ
%**************************************************************************
function DataOut = integral_time(DataIn, FreqTime, flag)

%��������ݵĳ�ʼ��
DataOut  =  zeros(size(DataIn));

switch flag 
    case 1   % ���ӷ�
        for i = 2:length(DataIn)
            DataOut(i) = DataOut(i-1) + DataIn(i) * FreqTime;
        end
    case 2   % ���η�
        for i = 2:length(DataIn)
            DataOut(i) = DataOut(i-1)+(DataIn(i-1) + DataIn(i))/2 * FreqTime;
        end
    case 3   % ����ɭ
        DataOut(1) = (DataIn(1)+DataIn(2))*FreqTime/2;
        for i = 2:length(DataIn)-1
            DataOut(i) = DataOut(i-1) + (DataIn(i-1) + 4 * DataIn(i) + DataIn(i+1)) * FreqTime / 6;           
        end
        DataOut(length(DataIn)) = DataOut(length(DataIn)-1);
end

end



