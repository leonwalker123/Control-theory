function [phi_theta] = Phi(theta)
% input:theta n*1维向量
% output:phi_theta 标量
eps = 0.08;%投影公差范围
%theta_max = max(theta);%投影范数界
theta_max = 0.5;
num = (eps+1)*(theta'*theta)-theta_max^2;
den = eps*(theta_max^2);
phi_theta = num/den;
end

