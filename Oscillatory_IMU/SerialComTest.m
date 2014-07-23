%���ڵĲ��Գ��� ò�ƴ��������� ����ʲôԭ�򻹲�֪��ѽ
close all;clear; clc;         

%����������ļ���
addpath('ximu_matlab_library');	addpath('quaternion_library');	
addpath('AHRS');                addpath('serials');

%
global ExpData;global OtrData;global ahrs;

ExpData.Gyro = [];
ExpData.Gacc = [];

OtrData.t    = [];
OtrData.ii   = 0;

View     = [30 20];

%% ����ͼ����ƵĴ���
FigHandle = figure('Number', 'off', 'Name', '6DOF Animation');
set(gca, 'drawmode', 'fast'); set(gcf, 'Renderer', 'zbuffer');
set(FigHandle, 'Position', [9 39 1280 720]);
lighting phong; hold on; axis equal; grid on;

%���ù۲��г�
view(View(1, 1), View(1, 2));

%����ԭ��
orgHandle = plot3(0, 0, 0, '--r*','EraseMode','background','MarkerSize',5);

%���Ʒ����ͷ
Xhandle  = quiver3(0, 0, 0, 1, 0, 0,  'r', 'ShowArrowHead', 'on', 'MaxHeadSize', 0.9, 'AutoScale', 'off');
Yhandle  = quiver3(0, 0, 0, 0, 1, 0,  'g', 'ShowArrowHead', 'on', 'MaxHeadSize', 0.9, 'AutoScale', 'off');
Zhandle  = quiver3(0, 0, 0, 0, 0, 1,  'b', 'ShowArrowHead', 'on', 'MaxHeadSize', 0.9, 'AutoScale', 'off');

XhandleB = quiver3(0, 0, 0, 1, 0, 0,  'k', 'ShowArrowHead', 'on', 'MaxHeadSize', 0.9, 'AutoScale', 'off');
YhandleB = quiver3(0, 0, 0, 0, 1, 0,  'k', 'ShowArrowHead', 'on', 'MaxHeadSize', 0.9, 'AutoScale', 'off');
ZhandleB = quiver3(0, 0, 0, 0, 0, 1,  'k', 'ShowArrowHead', 'on', 'MaxHeadSize', 0.9, 'AutoScale', 'off');

%���������������
axis([-1.5, 1.5, -1.5, 1.5, -1.5, 1.5]);

%% ���ô�������
% ���ô��ںţ��ַ�����ʽ
strSerialPortNum  = input('���ںţ����֣���');
strSerialPortName = ['COM',num2str(strSerialPortNum)];

% ���ô��ڲ��� ���ô��ڶ���
SerialCom = serial(strSerialPortName);

% ���ô��ڵ�����
SerialCom.Terminator        = 'CR';          %������ֹ��Ϊ CR���س�������ȱʡΪ LF�����з���
SerialCom.InputBufferSize   = 1024;          %���뻺����Ϊ 256B��ȱʡֵΪ 512B
SerialCom.OutputBufferSize  = 1024;          %���뻺����Ϊ 256B��ȱʡֵΪ 512B
SerialCom.Timeout           = 0.5;           %����һ�ζ���д������������ʱ��Ϊ 0.5s,ȱʡֵΪ 10s
SerialCom.ReadAsyncMode     ='continuous';
SerialCom.BaudRate          = 115200;
SerialCom.DataBits          = 8;
SerialCom.TimeOut           = 1;

% ���ûص�����
SerialCom.BytesAvailableFcn     = {@SerialCallback, orgHandle, Xhandle, Yhandle, Zhandle, XhandleB, YhandleB, ZhandleB};
SerialCom.BytesAvailableFcnMode = 'byte';

get(SerialCom)

%% �򿪴���
try
   fopen(SerialCom);
catch err
    fprintf('%s��ʧ��\n', strSerialPortName);
    return;
end

fprintf('%s�ɹ���\n', strSerialPortName);

pause; 

%% �رմ��ڳ��� 
fclose(SerialCom);  delete(SerialCom); clear SerialCom;    