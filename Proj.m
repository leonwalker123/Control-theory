function [output] = Proj(theta,y)
% 投影函数
% 输入
% theta: n*1维向量
% y: n*1维向量
% 输出
% output: n*1维向量
p1 = Phi(theta);%标量
p2 = diff_Phi(theta);%n*1向量
if p1<0
    output = y;
else
    if p2'*y <= 0
        output = y;
    else
        output = y-(p2*p2'*y)*p1/(p2'*p2);
    end
end
end

