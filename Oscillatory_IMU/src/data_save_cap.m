function final_data = data_save_cap(file_name, freq, save_file, save_sheet)
% 功  能：胶囊数据采集结果的转换，转换为TXT文件
% 参  数：(IN/输入参数) ：
%                       file_name     待处理的文本文件
%						freq          当前数据采集的频率
%                       save_file     进行保存的XSL文件名
%                       save_sheet    保存的文件sheet名
%          (OUT/输出参数)：
%                       final_data     进行数据转换后的结果
% 返回值：
% 备  注：1.目前胶囊采集的数据为10HZ
%         2.
%**************************************************************************

% 从胶囊的数据中读取加速度和陀螺仪的值
SrcData       = textread(file_name,'%d')';         %以字符形式打开文件
DataNums      = ceil((length(SrcData)) / 32);
SrcMatrix     = reshape(SrcData, 32, DataNums);
Accelerometer = zeros(DataNums, 3);
Gyroscope     = zeros(DataNums, 3);

for i = 1:DataNums
    
    %加速计  12个字节
    Accelerometer(i, 1) = merge_hl_data(SrcMatrix(8,  i), SrcMatrix(7,  i), 12); 
    Accelerometer(i, 2) = merge_hl_data(SrcMatrix(10, i), SrcMatrix(9,  i), 12);
    Accelerometer(i, 3) = merge_hl_data(SrcMatrix(12, i), SrcMatrix(11, i), 12);
    
    %陀螺仪  16个字节
    Gyroscope(i, 1) = merge_hl_data(SrcMatrix(18, i), SrcMatrix(19, i), 16);
    Gyroscope(i, 2) = merge_hl_data(SrcMatrix(20, i), SrcMatrix(21, i), 16);
    Gyroscope(i, 3) = merge_hl_data(SrcMatrix(22, i), SrcMatrix(24, i), 16);
        
end

final_data(1:DataNums,1)    = 0:freq:(DataNums-1)*freq;
final_data(1:DataNums,2:4)  = Accelerometer(1:DataNums,1:3);
final_data(1:DataNums,5:7)  = Gyroscope(1:DataNums,1:3);

xlswrite(save_file,final_data,save_sheet)

end

function result_data = merge_hl_data(high, low, bits)
tmpdata     = bitshift(high, 8) + low;
tmpdata     = dec2bin(tmpdata, 16);
tmpdata     = tmpdata(1:bits);
b           = fi(0,1,bits,0);
b.bin       = tmpdata;
result_data = double(b);
end