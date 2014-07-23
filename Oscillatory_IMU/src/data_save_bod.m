function data_save_bod(file_name, freq, save_file, save_sheet)
% 功  能：开发板数据采集结果的转换，转换TXT为EXCEL文件
% 参  数：(IN/输入参数) ：
%                       file_name     待处理的文本文件
%						freq          当前数据采集的频率
%                       save_file     进行保存的XSL文件名
%                       save_sheet    保存的文件sheet名
%          (OUT/输出参数)：
%                       final_data     进行数据转换后的结果
% 返回值：
% 备  注：1.目前开发板采集的数据为400HZ
%         2.
%**************************************************************************

if nargin < 4
    error('输入参数错误');
end

SrcData       = textread(file_name,'%s')';%以字符形式打开文件
DataNums      = ceil((length(SrcData)-600)/14);
Accelerometer = zeros(DataNums,3);
Gyroscope     = zeros(DataNums,3);

CutOffNum  = 300;               %截断数据长度（开始 结束）
SrcDataLen = length(SrcData);   %整个数据的长度
start_idx  = 0;

% 数据长度过小
if SrcDataLen < CutOffNum * 10
    return;
end

% 首先进行起始位置的查找
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

% 进行数据的转换
i = start_idx; num = 1;
while i < SrcDataLen - CutOffNum
    
    %加速计  12个字
    Accelerometer(num, 1) = merge_hl_data(SrcData(i + 2), SrcData(i + 1), 12); 
    Accelerometer(num, 2) = merge_hl_data(SrcData(i + 4), SrcData(i + 3), 12);
    Accelerometer(num, 3) = merge_hl_data(SrcData(i + 6), SrcData(i + 5), 12);
    
    i = i + 7;
    
    %陀螺仪  16个字
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
