clear all

load alpha_U_beta1.mat
load alpha_U_beta2.mat
alpha_U_beta1_Ext = cat(6,alpha_U_beta1{:});
alpha_U_beta2_Ext = cat(6,alpha_U_beta2{:});
clear alpha_U_beta1 alpha_U_beta2
alpha_U_beta1_Ext = flip(alpha_U_beta1_Ext,6);
alpha_U_beta2_Ext = flip(alpha_U_beta2_Ext,6);

alpha_U_beta1_Ext(:,:,:,:,:,end+1:end+2/0.25) = ...
    repmat(alpha_U_beta1_Ext(:,:,:,:,:,end),[1,1,1,1,1,2/0.25]);

alpha_U_beta2_Ext(:,:,:,:,:,end+1:end+2/0.25) = ...
    repmat(alpha_U_beta2_Ext(:,:,:,:,:,end),[1,1,1,1,1,2/0.25]);

save('alpha_U_beta_Ext.mat')



%alpha_U_beta1_Ext(:,:,:,:,:,end+1:end+T_MPC_horizon/tstep) = repmat(alpha_U_beta1_Ext(:,:,:,:,:,end),[1,1,1,1,1,T_MPC_horizon/tstep]);
%alpha_U_beta2_Ext(:,:,:,:,:,end+1:end+T_MPC_horizon/tstep) = repmat(alpha_U_beta2_Ext(:,:,:,:,:,end),[1,1,1,1,1,T_MPC_horizon/tstep]);