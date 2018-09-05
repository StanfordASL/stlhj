
% clear all
% global alpha_U_beta1 alpha_U_beta2 gmatOut1 gmatOut2 deriv1 deriv_R1 deriv_L1 deriv2 deriv_R2 deriv_L2
% load alpha_U_beta1.mat 
% load alpha_U_beta2.mat 
% load deriv2.mat 
% load deriv1.mat 
% load derivLR2.mat 
% load derivLR1.mat

rosinit('http://joe.local:11311');
%rosinit
global ctrlpub msg

ctrlpub = rospublisher('/ctrl_MATLAB','std_msgs/Float64MultiArray');
msg = rosmessage(ctrlpub);

rossubscriber('/StateSpace',@ctrl_for_joe);

function ctrl_for_joe(~,message)
global alpha_U_beta1 alpha_U_beta2 gmatOut1 gmatOut2 deriv1 deriv_R1 deriv_L1 deriv2 deriv_R2 deriv_L2
global ctrlpub msg
x = message.Data(1);%message.x;
y = message.Data(2);%message.y;
th = message.Data(3);%message.theta;
V = message.Data(4);%message.V;
y2 = message.Data(5);%message.y2;
t = message.Data(6);
tstep_sim = 0.25;
if t>12
    t = 12;
end
i = floor(t/tstep_sim); %which alpha_U_beta to evaluate

traj = [x,y,th,V,y2];
value1 = 0;
value2 = 0;
[~,value1] = eval_u(traj,gmatOut1,alpha_U_beta1{end-i}(:,:,:,:,:),alpha_U_beta1{end-i}(:,:,:,:,:),0,value1);
[~,value2] = eval_u(traj,gmatOut2,alpha_U_beta2{end-i}(:,:,:,:,:),alpha_U_beta2{end-i}(:,:,:,:,:),0,value2);

%compute gradient of value function and control action
if value1>value2
    [u,~,~] = eval_u_deriv(traj,gmatOut1,deriv1{3}(:,:,:,:,:,end-i),deriv1{4}(:,:,:,:,:,end-i),...
        deriv_R1{4}(:,:,:,:,:,end-i),deriv_L1{4}(:,:,:,:,:,end-i),0,value1);
else
    [u,~,~] = eval_u_deriv(traj,gmatOut2,deriv2{3}(:,:,:,:,:,end-i),deriv2{4}(:,:,:,:,:,end-i),...
        deriv_R2{4}(:,:,:,:,:,end-i),deriv_L2{4}(:,:,:,:,:,end-i),0,value2);
end
u = [u(2),u(1)]';
u(isnan(u)) = 0;
msg.Data = double(u);
send(ctrlpub,msg);

end

%rosshutdown

