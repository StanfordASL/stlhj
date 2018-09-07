clear all

load overtake_output.mat
overtake_output_Ext = cat(6,alpha_U_beta{:});
%clear alpha_U_beta1 alpha_U_beta2
overtake_output_Ext = flip(overtake_output_Ext,6);

overtake_output_Ext(:,:,:,:,:,end+1:end+2/0.25) = ...
    repmat(overtake_output_Ext(:,:,:,:,:,end),[1,1,1,1,1,2/0.25]);

save('overtake_output_Ext.mat', 'overtake_output_Ext','g','-v7')



%alpha_U_beta1_Ext(:,:,:,:,:,end+1:end+T_MPC_horizon/tstep) = repmat(alpha_U_beta1_Ext(:,:,:,:,:,end),[1,1,1,1,1,T_MPC_horizon/tstep]);
%alpha_U_beta2_Ext(:,:,:,:,:,end+1:end+T_MPC_horizon/tstep) = repmat(alpha_U_beta2_Ext(:,:,:,:,:,end),[1,1,1,1,1,T_MPC_horizon/tstep]);