function final_data = capluse_data_save(file_name, frequency, save_file, save_sheet)
% 从胶囊的数据中读取加速度和陀螺仪的值

SrcData       = textread(file_name,'%d')';%以字符形式打开文件
DataNums      = ceil((length(SrcData)) / 32);
SrcMatrix     = reshape(SrcData, 32, DataNums);
Accelerometer = zeros(DataNums, 3);
Gyroscope     = zeros(DataNums, 3);

for i = 1:DataNums
    
    %加速度计  
    Accelerometer(i, 1) = merge_hl_data(SrcMatrix(8,  i), SrcMatrix(7,  i), 12); 
    Accelerometer(i, 2) = merge_hl_data(SrcMatrix(10, i), SrcMatrix(9,  i), 12);
    Accelerometer(i, 3) = merge_hl_data(SrcMatrix(12, i), SrcMatrix(11, i), 12);
    
    %陀螺仪 
    Gyroscope(i, 1) = merge_hl_data(SrcMatrix(18, i), SrcMatrix(19, i), 16);
    Gyroscope(i, 2) = merge_hl_data(SrcMatrix(20, i), SrcMatrix(21, i), 16);
    Gyroscope(i, 3) = merge_hl_data(SrcMatrix(22, i), SrcMatrix(24, i), 16);
        
end

final_data(1:DataNums,1)    = 0:frequency:(DataNums-1)*frequency;
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