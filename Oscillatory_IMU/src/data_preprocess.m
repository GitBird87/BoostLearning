%**************************************************************************
% 功  能：对输出的数据进行预处理，包含去除零点偏差，Kalman滤波
% 参  数：(IN/输入参数) ：
%                       acc           输入的加速度
%						gyr           输入的陀螺仪
%                       draw_flg      是否绘制处理的结果
%                       acc_bias      加速度的零偏
%                       gyr_bias      陀螺仪的零偏
%          (OUT/输出参数)：
%                       acc_data      输出的加速度数据
%                       gyr_data      输出的陀螺仪数据
% 返回值：
% 备  注：方法比较简单，后续需要修改
%**************************************************************************
function [acc_data, gyr_data] = data_preprocess(acc, gyr, draw_flg, acc_bias, gyr_bias)

switch nargin
    case 2
        draw_flg = 0;
        acc_bias = zeros(1, 3);
        gyr_bias = zeros(1, 3);
    case 3
        acc_bias = zeros(1, 3);
        gyr_bias = zeros(1, 3);
    case 4
        gyr_bias = zeros(1, 3);
end

% 1. 进行数据的去除bias操作
if abs(acc_bias(3)) > 0.5
    acc_bias(3) = acc_bias(3) - 1;
end

for i = 1 : 3
    acc(:, i) = acc(:, i) - acc_bias(i); 
    gyr(:, i) = gyr(:, i) - gyr_bias(i); 
end

% 2. 进行Kalman滤波操作，去除高频噪声与干扰
gyrK = zeros(size(gyr)); accK = zeros(size(acc));
for i = 1:3
    gyrK(:, i) = kalman(gyr(:, i), gyr(1, i), 4e-4, 0.03);
    accK(:, i) = kalman(acc(:, i), acc(1, i), 4e-4, 0.03);
end

acc_data = accK;
gyr_data = gyrK;

if (draw_flg == 0)
    return;
end

% 3. 绘制预处理后的曲线
figure('Number', 'off', 'Name', 'Gyroscope');

subplot(2,1,1); hold on; xlabel('sample'); ylabel('dps');
plot(gyr(:,1), 'r');plot(gyr(:,2), 'g');plot(gyr(:,3), 'b');
title('Gyroscope');legend('X', 'Y', 'Z');

subplot(2,1,2); hold on; xlabel('sample'); ylabel('dps');
plot(gyrK(:,1), 'r');plot(gyrK(:,2), 'g');plot(gyrK(:,3), 'b');
title('GyroscopeK');legend('X', 'Y', 'Z');

figure('Number', 'off', 'Name', 'Accelerometer');

subplot(2,1,1); hold on;xlabel('sample');ylabel('g');
plot(acc(:,1), 'r');plot(acc(:,2), 'g');plot(acc(:,3), 'b');
title('Accelerometer');legend('X', 'Y', 'Z');

subplot(2,1,2); hold on;xlabel('sample');ylabel('g');
plot(accK(:,1), 'r');plot(accK(:,2), 'g');plot(accK(:,3), 'b');
title('AccelerometerK');legend('X', 'Y', 'Z');

end