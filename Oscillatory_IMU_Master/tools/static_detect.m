%**************************************************************************
% ��  �ܣ����ݵ�ǰ�ļ��ٶȷ��ȣ�ȷ��Ŀ��ľ�ֹʱ��
% ��  ����(IN/�������) ��
%                       magFlt        ���ٶȵķ���
%						thresh        ���ٶȵ���С��ֵ
%                       nums          ������ֹ�Ĳ�������
%          (OUT/�������)��
%                       lessMatix     ���������ֵ�ľ�����
% ����ֵ��
% ��  ע�������Ƚϼ򵥣�������Ҫ�޸�
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