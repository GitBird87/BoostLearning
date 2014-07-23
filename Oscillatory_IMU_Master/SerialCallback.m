function SerialCallback(SerialCom, BytesAvailable, FigHandle, quivXhandle, quivYhandle, quivZhandle, quivXhandleB, quivYhandleB, quivZhandleB)

global ExpData;
global OtrData;
global ahrs;

%%
% 读取串口的数据
ResultData = fscanf(SerialCom);
ResultData = str2num(ResultData);

% 串口数据读取成功
if length(ResultData) == 7
    
    % 进行offset数据校正
    ResultData(2) = ResultData(2) - 0.0034 * 1000.0;
    ResultData(3) = ResultData(3) + 0.0106 * 1000.0;
    ResultData(4) = ResultData(4) - 0.0211 * 1000.0 ;
    
    ResultData(5) = ResultData(5) - 0.3017 * 15.0;
    ResultData(6) = ResultData(6) + 0.1665 * 15.0;
    ResultData(7) = ResultData(7) - 0.2822 * 15.0;
    
    % 保存串口数据
    ExpData.Gacc = [ExpData.Gacc; ResultData(2:4) / 1000.0];
    ExpData.Gyro = [ExpData.Gyro; ResultData(5:7) / 15.0];
  
    OtrData.t    = [OtrData.t, OtrData.ii];
    OtrData.x    = OtrData.x  + 1;
    OtrData.ii   = OtrData.ii + 1; 
    
    %进行算法的处理
    ImuAlgProcess(FigHandle, quivXhandle, quivYhandle, quivZhandle, quivXhandleB, quivYhandleB, quivZhandleB);
    
end
end