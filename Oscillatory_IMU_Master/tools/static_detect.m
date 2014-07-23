%**************************************************************************
% 功  能：根据当前的加速度幅度，确定目标的静止时刻
% 参  数：(IN/输入参数) ：
%                       magFlt        加速度的幅度
%						thresh        加速度的最小阈值
%                       nums          连续静止的采样次数
%          (OUT/输出参数)：
%                       lessMatix     输出低于阈值的矩阵数
% 返回值：
% 备  注：方法比较简单，后续需要修改
%**************************************************************************
function staticMatix = static_detect(magFlt, thresh, nums)

sizex       = length(magFlt);
staticMatix = magFlt < thresh;
connum      = 0;

for i = 1 : sizex
   
    if (staticMatix(i) == 1)
        connum = connum + 1;
    else
        if(connum < nums)
            staticMatix(i - connum : i) = 0;
        end
        connum = 0;
    end
    
end

end