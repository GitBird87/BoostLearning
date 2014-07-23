classdef MahonyAHRS < handle
%MAYHONYAHRS Madgwick's implementation of Mayhony's AHRS algorithm
%
%   For more information see:
%   http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
%
%   Date          Author          Notes
%   28/09/2011    SOH Madgwick    Initial release
 
    %% Public properties
    properties (Access = public)
        SamplePeriod = 1/256;
        Quaternion = [1 0 0 0];     % output quaternion describing the Earth relative to the sensor
        Kp = 1;                     % algorithm proportional gain
        Ki = 0;                     % algorithm integral gain
    end
    
    %% Public properties
    properties (Access = private)
        eInt = [0 0 0];             % integral error
    end    
 
    %% Public methods
    methods (Access = public)
        
        %%
        function obj = MahonyAHRS(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod') 
                    obj.SamplePeriod = varargin{i+1};
                elseif strcmp(varargin{i}, 'Quaternion')
                    obj.Quaternion = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Kp')
                    obj.Kp = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Ki')
                    obj.Ki = varargin{i+1};
                else
                    error('Invalid argument');
                end
            end;
        end
        
       %% 利用加速度计 陀螺仪  磁力计进行速度的角度的计算
        function obj = Update(obj, Gyroscope, Accelerometer, Magnetometer)
            q = obj.Quaternion; % short name local variable for readability
 
            % Normalise accelerometer measurement
            if(norm(Accelerometer) == 0), return; end   % handle NaN
            Accelerometer = Accelerometer / norm(Accelerometer);    % normalise magnitude
 
            % Normalise magnetometer measurement
            if(norm(Magnetometer) == 0), return; end    % handle NaN
            Magnetometer = Magnetometer / norm(Magnetometer);   % normalise magnitude
 
            % Reference direction of Earth's magnetic feild
            h = quaternProd(q, quaternProd([0 Magnetometer], quaternConj(q)));
            b = [0 norm([h(2) h(3)]) 0 h(4)];
            
            % Estimated direction of gravity and magnetic field
            v = [2*(q(2)*q(4) - q(1)*q(3))
                 2*(q(1)*q(2) + q(3)*q(4))
                 q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2];
            w = [2*b(2)*(0.5 - q(3)^2 - q(4)^2) + 2*b(4)*(q(2)*q(4) - q(1)*q(3))
                 2*b(2)*(q(2)*q(3) - q(1)*q(4)) + 2*b(4)*(q(1)*q(2) + q(3)*q(4))
                 2*b(2)*(q(1)*q(3) + q(2)*q(4)) + 2*b(4)*(0.5 - q(2)^2 - q(3)^2)]; 
 
            % Error is sum of cross product between estimated direction and measured direction of fields
            e = cross(Accelerometer, v) + cross(Magnetometer, w); 
            if(obj.Ki > 0)
                obj.eInt = obj.eInt + e * obj.SamplePeriod;   
            else
                obj.eInt = [0 0 0];
            end
            
            % Apply feedback terms
            Gyroscope = Gyroscope + obj.Kp * e + obj.Ki * obj.eInt;            
            
            % Compute rate of change of quaternion
            qDot = 0.5 * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]);
 
            % Integrate to yield quaternion
            q = q + qDot * obj.SamplePeriod;
            obj.Quaternion = q / norm(q); % normalise quaternion
        end
        
       %%
        %根据陀螺仪和加速计更新当前的四元数
        function obj = UpdateIMU(obj, Gyroscope, Accelerometer)
            
            %n-1时刻的四元数
            q = obj.Quaternion;
 
            % 对加速计的值进行归一化处理
            if(norm(Accelerometer) == 0)
                return; 
            end   % handle NaN
            Accelerometer = Accelerometer / norm(Accelerometer);
 
            % Estimated direction of gravity and magnetic flux
            % 这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
            % 根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
            % 所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。
            v = [2*(q(2)*q(4) - q(1)*q(3))
                 2*(q(1)*q(2) + q(3)*q(4))
                 q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2];
 
            % Error is sum of cross product between estimated direction and measured direction of field
            % Accelerometer是机体坐标参照系上，加速度计测出来的重力向量，也就是实际测出来的重力向量。
            % Accelerometer是测量得到的重力向量，v是陀螺积分后的姿态来推算出的重力向量，它们都是机体坐标参照系上的重力向量。
            % 那它们之间的误差向量，就是陀螺积分后的姿态和加计测出来的姿态之间的误差。
            % 向量间的误差，可以用向量叉积（也叫向量外积、叉乘）来表示，exyz就是两个重力向量的叉积。
            % 这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。
            %（你可以自己拿东西想象一下）由于陀螺是对机体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的纠正。
            e = cross(Accelerometer, v); 
            
            %用叉积误差来做PI修正陀螺零偏
            if(obj.Ki > 0)
                obj.eInt = obj.eInt + e * obj.SamplePeriod;   
            else
                obj.eInt = [0 0 0];
            end
            
            % Apply feedback terms
            % 利用加速度对陀螺仪进行修正
            Gyroscope = Gyroscope + obj.Kp * e + obj.Ki * obj.eInt;            
            
            % 通过四元数的叉乘来计算标准坐标系下四元数的变化
            qDot = 0.5 * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]);
 
            % 考虑变化的周期，进行累加得到当前位置的四元数
            q = q + qDot * obj.SamplePeriod;
            obj.Quaternion = q / norm(q); % normalise quaternion
        end
        
    end
    
end