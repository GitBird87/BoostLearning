function data_save_bod(file_name, freq, save_file, save_sheet)
% ��  �ܣ����������ݲɼ������ת����ת��TXTΪEXCEL�ļ�
% ��  ����(IN/�������) ��
%                       file_name     ��������ı��ļ�
%						freq          ��ǰ���ݲɼ���Ƶ��
%                       save_file     ���б����XSL�ļ���
%                       save_sheet    ������ļ�sheet��
%          (OUT/�������)��
%                       final_data     ��������ת����Ľ��
% ����ֵ��
% ��  ע��1.Ŀǰ������ɼ�������Ϊ400HZ
%         2.
%**************************************************************************

if nargin < 4
    error('�����������');
end

SrcData       = textread(file_name,'%s')';%���ַ���ʽ���ļ�
DataNums      = ceil((length(SrcData)-600)/14);
Accelerometer = zeros(DataNums,3);
Gyroscope     = zeros(DataNums,3);

CutOffNum  = 300;               %�ض����ݳ��ȣ���ʼ ������
SrcDataLen = length(SrcData);   %�������ݵĳ���
start_idx  = 0;

% ���ݳ��ȹ�С
if SrcDataLen < CutOffNum * 10
    return;
end

% ���Ƚ�����ʼλ�õĲ���
for i = CutOffNum : SrcDataLen - CutOffNum
    
    index_g = zeros(1, 3);
    for j = 0 : 2
        index_g(j + 1) = strcmp(SrcData(i + j * 14), '33');
    end
    
    index_a = zeros(1, 3);
    for j = 0 : 2
        index_a(j + 1) = strcmp(SrcData(i + j * 14 + 7), 'B1');
    end
    
    if (sum(index_a) + sum(index_g) == 6)
        start_idx = i;
        break;
    end

end

if start_idx == 0
    error('src_data error');
end

% �������ݵ�ת��
i = start_idx; num = 1;
while i < SrcDataLen - CutOffNum
    
    %���ټ�  12����
    Accelerometer(num, 1) = merge_hl_data(SrcData(i + 2), SrcData(i + 1), 12); 
    Accelerometer(num, 2) = merge_hl_data(SrcData(i + 4), SrcData(i + 3), 12);
    Accelerometer(num, 3) = merge_hl_data(SrcData(i + 6), SrcData(i + 5), 12);
    
    i = i + 7;
    
    %������  16����
    Gyroscope(num, 1) = merge_hl_data(SrcData(i + 1), SrcData(i + 2), 16);
    Gyroscope(num, 2) = merge_hl_data(SrcData(i + 3), SrcData(i + 4), 16);
    Gyroscope(num, 3) = merge_hl_data(SrcData(i + 5), SrcData(i + 6), 16);
    
    i   = i   + 7;
    num = num + 1;
    
end

L = num - 1;
final_data(1:L,1)   = 0:freq:(L-1)*freq;
final_data(1:L,2:4) = Accelerometer(1:L,1:3);
final_data(1:L,5:7) = Gyroscope(1:L,1:3);

xlswrite(save_file,final_data,save_sheet)
end

%
function result_data = merge_hl_data(high, low, bits)
tmpdata     = bitshift(high, 8) + low;
tmpdata     = dec2bin(tmpdata, 16);
tmpdata     = tmpdata(1:bits);
b           = fi(0,1,bits,0);
b.bin       = tmpdata;
result_data = double(b);
end
