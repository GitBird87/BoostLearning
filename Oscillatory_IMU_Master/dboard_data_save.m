
%����ת��
%����Ϊ�ɼ��������Ǻͼ��ٶȼƵ�ԭʼ���ݱ����txt�ļ�
%���Ϊexcel�ļ�
function dboard_data_save(file_name,samp_frequency,save_file,save_sheet)

if nargin == 0
    file_name      = 'base1.txt';
    samp_frequency = 0.01;
    save_file      = 'datashift.xls';
    save_sheet     = 'sheet1';
else if nargin == 1
        samp_frequency=0.01;
        save_file='datashift.xls';
        save_sheet='sheet1';
    else if nargin == 2
            save_file='datashift.xls';
            save_sheet='sheet1';
        else if nargin == 3
                save_sheet='sheet1';
            end
        end
    end
end

SrcData       = textread(file_name,'%s')';%���ַ���ʽ���ļ�
DataNums      = ceil((length(SrcData)-600)/14);
Accelerometer = zeros(DataNums,3);
Gyroscope     = zeros(DataNums,3);

num        = 1;
i          = 1;
CutOffNum  = 300;               %�ض����ݳ��ȣ���ʼ ������
SrcDataLen = length(SrcData);   %�������ݵĳ���
start_idx  = 0;

% ���ݳ��ȹ�С
if SrcDataLen < CutOffNum * 10
    return;
end

%% ���Ƚ�����ʼλ�õĲ���
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
    disp('src_data error');
    return;
end

%% �������ݵ�ת��
i = start_idx;
while i < SrcDataLen - CutOffNum
    
    %���ٶȼ� 12bit
    TmpData = hex2dec(SrcData(i+1)) + bitshift(hex2dec(SrcData(i+2)), 8);
    a = dec2bin(TmpData, 16);
    a = a(1:12);
    b = fi(0,1,12,0);
    b.bin = a;
    Accelerometer(num,1) = b;
    
    TmpData=hex2dec(SrcData(i+3)) + bitshift(hex2dec(SrcData(i+4)),8);
    a = dec2bin(TmpData,16);
    a = a(1:12);
    b = fi(0,1,12,0);
    b.bin=a;
    Accelerometer(num,2)=double(b);
    
    TmpData=hex2dec(SrcData(i+5))+bitshift(hex2dec(SrcData(i+6)),8);
    a = dec2bin(TmpData,16);
    a = a(1:12);
    b = fi(0,1,12,0);
    b.bin=a;
    Accelerometer(num,3)=double(b);
    
    i = i + 7;
    
    % ������ 16bit
    TmpData = bitshift(hex2dec(SrcData(i+1)),8)+hex2dec(SrcData(i+2));
    a = dec2bin(TmpData);
    b = fi(0,1,16,0);
    b.bin=a;
    Gyroscope(num,1)=double(b);
    
    TmpData=bitshift(hex2dec(SrcData(i+3)),8)+hex2dec(SrcData(i+4));
    a = dec2bin(TmpData);
    b = fi(0,1,16,0);
    b.bin=a;
    Gyroscope(num,2)=double(b);
    
    TmpData=bitshift(hex2dec(SrcData(i+5)),8)+hex2dec(SrcData(i+6));
    a = dec2bin(TmpData);
    b = fi(0,1,16,0);
    b.bin=a;
    Gyroscope(num,3)=double(b);
    
    i   = i   + 7;
    num = num + 1;
    
end

L = num - 1;
final_data(1:L,1)   = 0:samp_frequency:(L-1)*samp_frequency;
final_data(1:L,2:4) = Accelerometer(1:L,1:3);
final_data(1:L,5:7) = Gyroscope(1:L,1:3);

xlswrite(save_file,final_data,save_sheet)
end

