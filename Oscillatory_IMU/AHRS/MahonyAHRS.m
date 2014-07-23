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
        
       %% ���ü��ٶȼ� ������  �����ƽ����ٶȵĽǶȵļ���
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
        %���������Ǻͼ��ټƸ��µ�ǰ����Ԫ��
        function obj = UpdateIMU(obj, Gyroscope, Accelerometer)
            
            %n-1ʱ�̵���Ԫ��
            q = obj.Quaternion;
 
            % �Լ��ټƵ�ֵ���й�һ������
            if(norm(Accelerometer) == 0)
                return; 
            end   % handle NaN
            Accelerometer = Accelerometer / norm(Accelerometer);
 
            % Estimated direction of gravity and magnetic flux
            % ���ǰ���Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء�
            % �������Ҿ����ŷ���ǵĶ��壬��������ϵ������������ת����������ϵ��������������Ԫ�ء�
            % ���������vx\y\z����ʵ���ǵ�ǰ��ŷ���ǣ�����Ԫ�����Ļ����������ϵ�ϣ����������������λ������
            v = [2*(q(2)*q(4) - q(1)*q(3))
                 2*(q(1)*q(2) + q(3)*q(4))
                 q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2];
 
            % Error is sum of cross product between estimated direction and measured direction of field
            % Accelerometer�ǻ����������ϵ�ϣ����ٶȼƲ����������������Ҳ����ʵ�ʲ����������������
            % Accelerometer�ǲ����õ�������������v�����ݻ��ֺ����̬����������������������Ƕ��ǻ����������ϵ�ϵ�����������
            % ������֮�������������������ݻ��ֺ����̬�ͼӼƲ��������̬֮�����
            % ������������������������Ҳ�������������ˣ�����ʾ��exyz�����������������Ĳ����
            % �����������Ծ���λ�ڻ�������ϵ�ϵģ������ݻ������Ҳ���ڻ�������ϵ�����Ҳ���Ĵ�С�����ݻ����������ȣ����������������ݡ�
            %��������Լ��ö�������һ�£����������ǶԻ���ֱ�ӻ��֣����Զ����ݵľ�������ֱ�������ڶԻ�������ϵ�ľ�����
            e = cross(Accelerometer, v); 
            
            %�ò���������PI����������ƫ
            if(obj.Ki > 0)
                obj.eInt = obj.eInt + e * obj.SamplePeriod;   
            else
                obj.eInt = [0 0 0];
            end
            
            % Apply feedback terms
            % ���ü��ٶȶ������ǽ�������
            Gyroscope = Gyroscope + obj.Kp * e + obj.Ki * obj.eInt;            
            
            % ͨ����Ԫ���Ĳ���������׼����ϵ����Ԫ���ı仯
            qDot = 0.5 * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]);
 
            % ���Ǳ仯�����ڣ������ۼӵõ���ǰλ�õ���Ԫ��
            q = q + qDot * obj.SamplePeriod;
            obj.Quaternion = q / norm(q); % normalise quaternion
        end
        
    end
    
end