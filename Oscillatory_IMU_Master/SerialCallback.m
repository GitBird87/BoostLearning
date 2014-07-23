function SerialCallback(SerialCom, BytesAvailable, FigHandle, quivXhandle, quivYhandle, quivZhandle, quivXhandleB, quivYhandleB, quivZhandleB)

global ExpData;
global OtrData;
global ahrs;

%%
% ��ȡ���ڵ�����
ResultData = fscanf(SerialCom);
ResultData = str2num(ResultData);

% �������ݶ�ȡ�ɹ�
if length(ResultData) == 7
    
    % ����offset����У��
    ResultData(2) = ResultData(2) - 0.0034 * 1000.0;
    ResultData(3) = ResultData(3) + 0.0106 * 1000.0;
    ResultData(4) = ResultData(4) - 0.0211 * 1000.0 ;
    
    ResultData(5) = ResultData(5) - 0.3017 * 15.0;
    ResultData(6) = ResultData(6) + 0.1665 * 15.0;
    ResultData(7) = ResultData(7) - 0.2822 * 15.0;
    
    % ���洮������
    ExpData.Gacc = [ExpData.Gacc; ResultData(2:4) / 1000.0];
    ExpData.Gyro = [ExpData.Gyro; ResultData(5:7) / 15.0];
  
    OtrData.t    = [OtrData.t, OtrData.ii];
    OtrData.x    = OtrData.x  + 1;
    OtrData.ii   = OtrData.ii + 1; 
    
    %�����㷨�Ĵ���
    ImuAlgProcess(FigHandle, quivXhandle, quivYhandle, quivZhandle, quivXhandleB, quivYhandleB, quivZhandleB);
    
end
end