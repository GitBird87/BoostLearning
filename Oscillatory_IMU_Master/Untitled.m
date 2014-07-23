addpath('ximu_matlab_library');	% include x-IMU MATLAB library
addpath('quaternion_library');	% include quatenrion library
close all;                     	% close all figures
clear;                         	% clear all variables
clc;                          	% clear the command terminal

global ExpData;
global OtrData;
global ahrs;

ExpData.Gyro = [];
ExpData.Gacc = [];

OtrData.t     = [];
OtrData.ii    = 0;
OtrData.x     = -100;

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
quivXhandle = quiver3(0, 0, 0, 1, 0, 0,  'r', 'ShowArrowHead', 'on', 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
quivYhandle = quiver3(0, 0, 0, 0, 1, 0,  'g', 'ShowArrowHead', 'on', 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
quivZhandle = quiver3(0, 0, 0, 0, 0, 1,  'b', 'ShowArrowHead', 'on', 'MaxHeadSize', 0.999999, 'AutoScale', 'off');

quivXhandleB = quiver3(0, 0, 0, 1, 0, 0,  'k', 'ShowArrowHead', 'on', 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
quivYhandleB = quiver3(0, 0, 0, 0, 1, 0,  'k', 'ShowArrowHead', 'on', 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
quivZhandleB = quiver3(0, 0, 0, 0, 0, 1,  'k', 'ShowArrowHead', 'on', 'MaxHeadSize', 0.999999, 'AutoScale', 'off');

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
SerialCom.Timeout           = 0.01;          %����һ�ζ���д������������ʱ��Ϊ 0.5s,ȱʡֵΪ 10s
SerialCom.ReadAsyncMode     ='continuous';
SerialCom.BaudRate          = 115200;
SerialCom.DataBits          = 8;
SerialCom.TimeOut           = 1;

% ���ûص�����
SerialCom.BytesAvailableFcn     = {@SerialCallback, orgHandle, quivXhandle, quivYhandle, quivZhandle, quivXhandleB, quivYhandleB, quivZhandleB};
SerialCom.BytesAvailableFcnMode = 'byte';

get(SerialCom)

%% �򿪴���
try
   fopen(SerialCom);
catch err
    fprintf('%s��ʧ�ܡ�\n', strSerialPortName);
    return;
end

fprintf('%s�ɹ��򿪡�\n', strSerialPortName);

pause;

%% �رմ��ڳ��� 
fclose(SerialCom);  %�رմ����豸����
delete(SerialCom);  %ɾ���ڴ��еĴ����豸����
clear SerialCom;    %��������ռ��еĴ����豸����