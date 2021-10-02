% author:Lw
clc
clear;
close all;
% 仿真时间设置
dt = 0.001;
Total = 1;
t = 0:dt:Total;

Num = length(t);

% 系统模型：航天器的横向动力学模型
% A:状态转移方程 B:控制方程
A = [-0.025, 0.104, -0.994;
    574.7, 0, 0;
    16.20, 0, 0];%3*3,n=3
B = [0.122, -0.276;
    -53.61, 33.25;
    195.5, -529.4];%3*2,m=2
% rank([B,A*B,A^2*B])=3，系统可控
% 系统状态(3*Num)
x = zeros(3,Num);
x_tilde = zeros(3,Num);
x(:,1) = [1;-2;-1];

u = zeros(2,Num);
u_tilde = zeros(2,Num);
K = [2.053, 0.079, -0.045;
    -3.823, -0.128, 0.102];%状态反馈控制增益
Ar = A+B*K;%闭环系统矩阵 
P = lyap(Ar',eye(3))
%P =   [47.2435    1.3757   -0.2152; 1.3757    0.0707   -0.0013;-0.2152   -0.0013    0.0114];
   
   
%使用place函数配置出的系统反馈矩阵
% K = -[-15.1475   -0.9053   -0.0612;-3.3565   -0.3342   -0.0858];   

% %使用icare求解Riccati方程获得的反馈增益与对应的矩阵P
% K = -[-15.6482   -1.0305    0.2960;9.8680   -0.0086   -0.9633];     
% P = [15.0874    0.2978   -0.0078;0.2978    0.0252    0.0014;-0.0078    0.0014    0.0019];   

% % Case1 标称模型状态反馈控制
% for k=1:(Num-1)
%     u(:,k) = K*x(:,k);
%     dx(:,k) = A*x(:,k)+B*u(:,k);
%     x(:,k+1)=x(:,k)+dx(:,k)*dt;
% end
% subplot(2,1,1)
% plot(t,25*x(1,:),'b',t,x(2,:),'--r',t,x(3,:),':k','linewidth',1.25);
% title('标称模型状态曲线(无攻击情形)');
% legend('25x_1','x_2','x_3');
% subplot(2,1,2)
% plot(t,u(1,:),'b',t,u(2,:),'--r','linewidth',1.25);
% title('标称模型控制输入');
% legend('u_1','u_2');
%%-------------------------------------------------------------------------

% 攻击参数设定
omega = -(0.75+0.15*sin(2.5*t));%注意omega>-1
delta_s = zeros(3,Num);%传感器通道攻击向量
delta_a = zeros(2,Num);%执行器通道攻击向量

% % Case2 攻击情形下标称控制器的控制效果
% for i=1:(Num-1)
%     delta_s(:,i) = omega(i)*x(:,i);
%     x_tilde(:,i) = x(:,i)+delta_s(:,i);
%     delta_a(:,i) = [1,1]'*0.5*cos(2.5*i)+[0.1*cos(2*i),0.5*sin(i)]'*0.2*sin(x(1,i))*cos(x(2,i));
%     u(:,i) = K*x_tilde(:,i);
%     u_tilde(:,i) = u(:,i)+delta_a(:,i);
%     dx(:,i) = A*x(:,i)+B*u_tilde(:,i);%玄学？若将此处的x(:,i)改为x_tilde(:,i)则无法复现论文中的结果
%     x(:,i+1) = x(:,i)+dx(:,i)*dt;
% end
% subplot(2,1,1)
% plot(t,25*x(1,:),'b',t,x(2,:),'--r',t,x(3,:),':k','linewidth',1.25);
% title('攻击情形下状态曲线');
% legend('25x_1','x_2','x_3');
% subplot(2,1,2)
% plot(t,u(1,:),'b',t,u(2,:),'--r','linewidth',1.25);
% title('攻击情形下控制输入');
% legend('u_1','u_2');
%%-------------------------------------------------------------------------
  
% %% case3 攻击情形下自适应校正控制器v(t)的设计
% v = zeros(2,Num);%v(t)维数与u(t)维数相同
% mu_hat = zeros(1,Num);%mu_hat为标量
% mu_hat(:,1) = 0.01;
% W_hat = zeros(1,2,Num);%W_hat为1*2矩阵
% W_hat(:,:,1) = [0.01,0.01];
% sigma_hat = zeros(1,Num);%sigma_hat为标量
% sigma_hat(1,1) = 0.01;
% %注意自适应估计参数的初始值
% gamma = 0.8;
% xi = 0.8;
% eta = 0.8;
% nu = 0.8;
% 
% %% 状态方程更新
% for i=1:(Num-1)
%     delta_s(:,i) = omega(i)*x(:,i);
%     x_tilde(:,i) = x(:,i)+delta_s(:,i);
%     delta_a(:,i) = [1,1]'*0.5*cos(2.5*i)+[0.1*cos(2*i),0.5*sin(i)]'*0.2*sin(x(1,i))*cos(x(2,i));
%     
%     % 自适应参数更新率
%     v(:,i) = -mu_hat(:,i)*K*x_tilde(:,i)-W_hat(:,:,i)'*0.2*sin(x(1,i))*cos(x(2,i)) -sigma_hat(:,i)*sgn(B'*P*x_tilde(:,i));
%     dmu_hat(:,i) = gamma*Proj(mu_hat(:,i),x_tilde(:,i)'*P*B*K*x_tilde(:,i));
%     temp(:,:,i) = 0.2*sin(x(1,i))*cos(x(2,i))*x_tilde(:,i)'*P*B;%1*2维
%     dW_hat(:,:,i) = eta*[Proj(W_hat(1,1,i),temp(1,1,i)),Proj(W_hat(1,2,i),temp(1,2,i))];
%     dsigma_hat(:,i) = nu*Proj(sigma_hat(:,i),sum(abs(x_tilde(:,i)'*P*B)));
%     mu_hat(:,i+1) = mu_hat(:,i)+dmu_hat(:,i)*dt;
%     W_hat(:,:,i+1) = W_hat(:,:,i)+dW_hat(:,:,i)*dt;
%     sigma_hat(:,i+1) = sigma_hat(:,i)+dsigma_hat(:,i)*dt;
%     
%     u(:,i) = K*x_tilde(:,i)+v(:,i);
%     u_tilde(:,i) = u(:,i)+delta_a(:,i);
%     dx(:,i) = A*x(:,i)+B*u_tilde(:,i);
%     x(:,i+1) = x(:,i)+dx(:,i)*dt;
% end
% figure(1)
% subplot(2,1,1)
% plot(t,25*x(1,:),'b',t,x(2,:),'--r',t,x(3,:),':k','linewidth',1.25);
% title('攻击情形下状态曲线');
% legend('25x_1','x_2','x_3');
% subplot(2,1,2)
% plot(t,u(1,:),'b',t,u(2,:),'--r','linewidth',1.25);
% title('攻击情形下控制输入');
% legend('u_1','u_2');
% WW_hat = reshape(W_hat,2,Num);
% figure(2)
% plot(t,mu_hat,'b',t,WW_hat(1,:),'--r',t,WW_hat(2,:),':k',t,sigma_hat,'-.g','linewidth',1.25);
% legend('$\hat{\mu}$','$\hat{W(1,:)}$','$\hat{W(2,:)}$'...
%         ,'$\hat{\sigma}$','Interpreter',"latex",'Fontsize',12);
% title('自适应参数估计值');
% %--------------------------------------------------------------------------
% 
% 
%% case4 使用tanh函数来消除sgn函数中的抖震现象
v = zeros(2,Num);%v(t)维数与u(t)维数相同
mu_hat = zeros(1,Num);%mu_hat为标量
mu_hat(:,1) = 0.01;
W_hat = zeros(1,2,Num);%W_hat为1*2矩阵
W_hat(:,:,1) = [0.01,0.01];
sigma_hat = zeros(1,Num);%sigma_hat为标量
sigma_hat(1,1) = 0.01;
%注意自适应估计参数的初始值
gamma = 0.8;
xi = 0.8;
eta = 0.8;
nu = 0.8;
% gamma = 2;
% xi = 2;
% eta = 2;
% nu = 2;

%% 状态方程更新
for i=1:(Num-1)
    delta_s(:,i) = omega(i)*x(:,i);
    x_tilde(:,i) = x(:,i)+delta_s(:,i);
    delta_a(:,i) = [1,1]'*0.5*cos(2.5*i)+[0.1*cos(2*i),0.5*sin(i)]'*0.2*sin(x(1,i))*cos(x(2,i));
    
    v(:,i) = -mu_hat(:,i)*K*x_tilde(:,i)-W_hat(:,:,i)'*0.5*cos(2.5*i) -sigma_hat(:,i)*tanh((B'*P*x_tilde(:,i)*sigma_hat(:,i))/0.8);
    dmu_hat(:,i) = gamma*Proj(mu_hat(:,i),x_tilde(:,i)'*P*B*K*x_tilde(:,i));
    temp(:,:,i) = 0.5*cos(2.5*i)*x_tilde(:,i)'*P*B;%1*2维
    dW_hat(:,:,i) = eta*[Proj(W_hat(1,1,i),temp(1,1,i)),Proj(W_hat(1,2,i),temp(1,2,i))];
    dsigma_hat(:,i) = nu*Proj(sigma_hat(:,i),sum(abs(x_tilde(:,i)'*P*B)));
    mu_hat(:,i+1) = mu_hat(:,i)+dmu_hat(:,i)*dt;
    W_hat(:,:,i+1) = W_hat(:,:,i)+dW_hat(:,:,i)*dt;
    sigma_hat(:,i+1) = sigma_hat(:,i)+dsigma_hat(:,i)*dt;
    
    u(:,i) = K*x_tilde(:,i)+v(:,i);
    u_tilde(:,i) = u(:,i)+delta_a(:,i);
    dx(:,i) = A*x(:,i)+B*u_tilde(:,i);%玄学？若将此处的x(:,i)改为x_tilde(:,i)则无法复现论文中的结果
    x(:,i+1) = x(:,i)+dx(:,i)*dt;
end
figure(1)
subplot(2,1,1)
plot(t,25*x(1,:),'b',t,x(2,:),'--r',t,x(3,:),':k','linewidth',1.25);
title('攻击情形下状态曲线');
legend('x_1*25','x_2','x_3');
subplot(2,1,2)
plot(t,u(1,:),'b',t,u(2,:),'--r','linewidth',1.25);
title('攻击情形下控制输入');
legend('u_1','u_2');
WW_hat = reshape(W_hat,2,Num);
figure(2)
plot(t,mu_hat,'b',t,WW_hat(1,:),'--r',t,WW_hat(2,:),':k',t,sigma_hat,'-.g','linewidth',1.25);
legend('$\hat{\mu}$','$\hat{W(1,:)}$','$\hat{W(2,:)}$'...
        ,'$\hat{sigma}$','Interpreter',"latex",'Fontsize',12);
title('自适应参数估计值');
    

