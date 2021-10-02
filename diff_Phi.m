function [output1] = diff_Phi(theta)  
% 输入:
% theta:n*1维向量
% 输出：
% 标量对向量求导，结果为n*1向量
eps = 0.8;%投影公差范围
theta_max = max(theta);%投影范数界
num = 2*(eps+1)*theta;
den = eps*(theta_max^2);
output1 = num/den;
end

