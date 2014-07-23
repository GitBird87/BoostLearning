%**************************************************************************
% ��  �ܣ�������̬��⣬�������ݽ��д��ݺͻ���
% ��  ����(IN/�������) ��
%                       FigHandle                    ����ͼ��ľ��
%						xhandle/yhandle/zhandle      XYZ���ݾ��
%                       xhandleB/yhandleB/zhandleB   XYZ��׼���ݾ�� 
%          (OUT/�������)��
% ����ֵ����
% ��  ע��
%**************************************************************************
function ImuAlgProcess(FigHandle, xhandle, yhandle, zhandle, xhandleB, yhandleB, zhandleB)

global ahrs;
global ExpData;
global OtrData;

addpath('AHRS')

srcFreq   = 100; downFreq = 1; 
curFreq   = srcFreq / downFreq;              %��ǰ��Ƶ��
curPeriod = 1 / curFreq;                     %��ǰ������
RotMatix  = eye(3, 3); euler = zeros(1 , 3); 

% û������ֱ�ӷ���
if (OtrData.ii == 0)
    return;
end

% ��ʼ����̬������
if (OtrData.ii == 1)
    ahrs  = MahonyAHRS('SamplePeriod', curPeriod, 'Kp', 1, 'Ki', 0.05); 
    %ahrs = MadgwickAHRS('SamplePeriod', curPeriod, 'Beta', 0.1);
end

% ���Ͳ����ʷ���
%if (mod(OtrData.ii, downFreq) == 0)
    
    %���㵱ǰ����̬ �����ǵ�����Ϊ���� ���������Ǻͼ��ټƵ�ֵ�����㵱ǰ����Ԫ��
    ahrs.UpdateIMU(ExpData.Gyro(OtrData.ii,:) * (pi/180), ExpData.Gacc(OtrData.ii,:));
    
    % ����Ԫ��ת��Ϊ��ת����
    RotMatix = quatern2rotMat(ahrs.Quaternion)';
    euler    = quatern2euler(ahrs.Quaternion) * 180 / pi;
    
    % ������ٶ� ����λ��
    
%end

% ���ݽǶȽ��л���
ox = 0; oy = 0; oz = 0;
ux = RotMatix(1, 1); vx = RotMatix(2, 1); wx = RotMatix(3, 1);
uy = RotMatix(1, 2); vy = RotMatix(2, 2); wy = RotMatix(3, 2);
uz = RotMatix(1, 3); vz = RotMatix(2, 3); wz = RotMatix(3, 3);

if (mod(OtrData.ii, 5) == 0)
    set(xhandle,  'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', ux, 'vdata', vx, 'wdata', wx);
    set(yhandle,  'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', uy, 'vdata', vy, 'wdata', wy);
    set(zhandle,  'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', uz, 'vdata', vz, 'wdata', wz);
    
    set(xhandleB, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', 1,  'vdata', 0,  'wdata', 0);
    set(yhandleB, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', 0,  'vdata', 1,  'wdata', 0);
    set(zhandleB, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', 0,  'vdata', 0,  'wdata', 1);
    
    str = sprintf('%d, %3.2f, %3.2f, %3.2f\n', OtrData.ii, euler(1), euler(2), euler(3));
    disp(str)
    drawnow
end

end