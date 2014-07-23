function ImuAlgProcess(FigHandle, quivXhandle, quivYhandle, quivZhandle, quivXhandleB, quivYhandleB, quivZhandleB)

global ahrs;
global ExpData;
global OtrData;

RotMatix     = eye(3, 3);
sampleRate   = 50;
samplePeriod = 1/105 * sampleRate;
euler        = zeros(1 , 3);

%% 没有数据直接返回
if (OtrData.ii == 0)
    return;
end

%% 初始化姿态计算类
if (OtrData.ii == 1)
    %ahrs  = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1, 'Ki', 0.05); 
    ahrs = MadgwickAHRS('SamplePeriod', samplePeriod, 'Beta', 0.1);
end

%% 降低采样率分析
if (mod(OtrData.ii, sampleRate) == 0)
    
    %计算当前的姿态 陀螺仪的输入为弧度 根据陀螺仪和加速计的值，计算当前的四元数
    ahrs.UpdateIMU(ExpData.Gyro(OtrData.ii,:) * (pi/180), ExpData.Gacc(OtrData.ii,:));
    
    % 将四元数转化为旋转矩阵 这里面的计算和书本上的公式不一致
    RotMatix = quatern2rotMat(ahrs.Quaternion)';
    euler    = quatern2euler(ahrs.Quaternion) * 180 / pi;
    
    % 计算加速度 计算位移
    
end

%% 根据角度进行绘制
ox = 0; oy = 0; oz = 0;
ux = RotMatix(1, 1); vx = RotMatix(2, 1); wx = RotMatix(3, 1);
uy = RotMatix(1, 2); vy = RotMatix(2, 2); wy = RotMatix(3, 2);
uz = RotMatix(1, 3); vz = RotMatix(2, 3); wz = RotMatix(3, 3);

if (mod(OtrData.ii, max(30, sampleRate)) == 0)
    set(quivXhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', ux, 'vdata', vx, 'wdata', wx);
    set(quivYhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', uy, 'vdata', vy, 'wdata', wy);
    set(quivZhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', uz, 'vdata', vz, 'wdata', wz);
    
    set(quivXhandleB, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', 1, 'vdata', 0, 'wdata', 0);
    set(quivYhandleB, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', 0, 'vdata', 1, 'wdata', 0);
    set(quivZhandleB, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', 0, 'vdata', 0, 'wdata', 1);
    
    str = sprintf('%d, %3.2f, %3.2f, %3.2f\n', OtrData.ii, euler(1), euler(2), euler(3));
    disp(str)
    drawnow
end

end