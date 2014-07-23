%串口的测试程序 貌似存在问题了 具体什么原因还不知道呀
close all;clear; clc;         

%加入包含的文件夹
addpath('ximu_matlab_library');	addpath('quaternion_library');	
addpath('AHRS');                addpath('serials');

%
global ExpData;global OtrData;global ahrs;

ExpData.Gyro = [];
ExpData.Gacc = [];

OtrData.t    = [];
OtrData.ii   = 0;

View     = [30 20];

%% 创建图像绘制的窗口
FigHandle = figure('Number', 'off', 'Name', '6DOF Animation');
set(gca, 'drawmode', 'fast'); set(gcf, 'Renderer', 'zbuffer');
set(FigHandle, 'Position', [9 39 1280 720]);
lighting phong; hold on; axis equal; grid on;

%设置观测市场
view(View(1, 1), View(1, 2));

%绘制原点
orgHandle = plot3(0, 0, 0, '--r*','EraseMode','background','MarkerSize',5);

%绘制方向箭头
Xhandle  = quiver3(0, 0, 0, 1, 0, 0,  'r', 'ShowArrowHead', 'on', 'MaxHeadSize', 0.9, 'AutoScale', 'off');
Yhandle  = quiver3(0, 0, 0, 0, 1, 0,  'g', 'ShowArrowHead', 'on', 'MaxHeadSize', 0.9, 'AutoScale', 'off');
Zhandle  = quiver3(0, 0, 0, 0, 0, 1,  'b', 'ShowArrowHead', 'on', 'MaxHeadSize', 0.9, 'AutoScale', 'off');

XhandleB = quiver3(0, 0, 0, 1, 0, 0,  'k', 'ShowArrowHead', 'on', 'MaxHeadSize', 0.9, 'AutoScale', 'off');
YhandleB = quiver3(0, 0, 0, 0, 1, 0,  'k', 'ShowArrowHead', 'on', 'MaxHeadSize', 0.9, 'AutoScale', 'off');
ZhandleB = quiver3(0, 0, 0, 0, 0, 1,  'k', 'ShowArrowHead', 'on', 'MaxHeadSize', 0.9, 'AutoScale', 'off');

%设置坐标轴的坐标
axis([-1.5, 1.5, -1.5, 1.5, -1.5, 1.5]);

%% 设置串口属性
% 设置串口号，字符串形式
strSerialPortNum  = input('串口号（数字）：');
strSerialPortName = ['COM',num2str(strSerialPortNum)];

% 设置串口参数 设置串口对象
SerialCom = serial(strSerialPortName);

% 设置串口的属性
SerialCom.Terminator        = 'CR';          %设置终止符为 CR（回车符），缺省为 LF（换行符）
SerialCom.InputBufferSize   = 1024;          %输入缓冲区为 256B，缺省值为 512B
SerialCom.OutputBufferSize  = 1024;          %输入缓冲区为 256B，缺省值为 512B
SerialCom.Timeout           = 0.5;           %设置一次读或写操作的最大完成时间为 0.5s,缺省值为 10s
SerialCom.ReadAsyncMode     ='continuous';
SerialCom.BaudRate          = 115200;
SerialCom.DataBits          = 8;
SerialCom.TimeOut           = 1;

% 设置回调函数
SerialCom.BytesAvailableFcn     = {@SerialCallback, orgHandle, Xhandle, Yhandle, Zhandle, XhandleB, YhandleB, ZhandleB};
SerialCom.BytesAvailableFcnMode = 'byte';

get(SerialCom)

%% 打开串口
try
   fopen(SerialCom);
catch err
    fprintf('%s打开失败\n', strSerialPortName);
    return;
end

fprintf('%s成功打开\n', strSerialPortName);

pause; 

%% 关闭串口程序 
fclose(SerialCom);  delete(SerialCom); clear SerialCom;    