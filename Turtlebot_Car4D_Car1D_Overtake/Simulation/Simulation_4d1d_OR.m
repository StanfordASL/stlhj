load alpha_U_beta1.mat
alpha_U_beta1 = alpha_U_beta;
save('alpha_U_beta1.mat','alpha_U_beta1','g')
%%
load alpha_U_beta2.mat
alpha_U_beta2 = alpha_U_beta;
save('alpha_U_beta2.mat','alpha_U_beta2','g')

%%
clear all
load alpha_U_beta1.mat
[gmat,datamat] = cpp2matG(g,alpha_U_beta1);
gmatOut1 = processGrid(gmat,datamat(:,:,:,:,:,1));

deriv_3 = computeGradients_C(gmatOut1,single(datamat),[0,0,1,0,0]);
deriv_4 = computeGradients_C(gmatOut1,single(datamat),[0,0,0,1,0]);
deriv_3{4} = deriv_4{4};
deriv1 = deriv_3;

save('deriv1.mat','deriv1','gmatOut1')
%%

clear all
load alpha_U_beta2.mat
[gmat,datamat] = cpp2matG(g,alpha_U_beta2);
gmatOut2 = processGrid(gmat,datamat(:,:,:,:,:,1));

deriv_3 = computeGradients_C(gmatOut2,single(datamat),[0,0,1,0,0]);
deriv_4 = computeGradients_C(gmatOut2,single(datamat),[0,0,0,1,0]);
deriv_3{4} = deriv_4{4};
deriv2 = deriv_3;

save('deriv2.mat','deriv2','gmatOut2')

%%
clear all
load alpha_U_beta1.mat
[gmat,datamat] = cpp2matG(g,alpha_U_beta1);
gmatOut1 = processGrid(gmat,datamat(:,:,:,:,:,1));

%[~,deriv_3_L,~] = computeGradients_L(gmatOut,single(datamat),[0,0,1,0,0]);
deriv_4_L = computeGradients_L(gmatOut1,single(datamat),[0,0,0,1,0]);
%deriv_3_L{4} = deriv_4_L{4};
deriv_L1 = deriv_4_L;


%deriv_3_R = computeGradients_R(gmatOut,single(datamat),[0,0,1,0,0]);
deriv_4_R = computeGradients_R(gmatOut1,single(datamat),[0,0,0,1,0]);
%deriv_3_R{4} = deriv_4_L{4};
deriv_R1 = deriv_4_R;

save('derivLR1.mat','deriv_L1','deriv_R1','gmatOut1')

%%
clear all
load alpha_U_beta2.mat
[gmat,datamat] = cpp2matG(g,alpha_U_beta2);
gmatOut2 = processGrid(gmat,datamat(:,:,:,:,:,1));

%[~,deriv_3_L,~] = computeGradients_L(gmatOut,single(datamat),[0,0,1,0,0]);
deriv_4_L = computeGradients_L(gmatOut2,single(datamat),[0,0,0,1,0]);
%deriv_3_L{4} = deriv_4_L{4};
deriv_L2 = deriv_4_L;


%deriv_3_R = computeGradients_R(gmatOut,single(datamat),[0,0,1,0,0]);
deriv_4_R = computeGradients_R(gmatOut2,single(datamat),[0,0,0,1,0]);
%deriv_3_R{4} = deriv_4_L{4};
deriv_R2 = deriv_4_R;

save('derivLR2.mat','deriv_L2','deriv_R2','gmatOut2')

%%
clear all
% close all
load alpha_U_beta1.mat
load deriv1.mat
load derivLR1.mat
load alpha_U_beta2.mat
load deriv2.mat
load derivLR2.mat
%%
T = 12; %total time
tstep = T/(length(alpha_U_beta1)-1); %timestep between aUb frames
tstep_sim = 0.125/2; %timestep between control evaluations
t_real = 0:tstep:T; 
u = zeros(tstep/tstep_sim,3);

traj = cell(numel(t_real),1);
%traj_temp = [0.2,-0.6,pi/2,0.15,0.5]; %no pass
traj_temp = [0.2,-0.4,pi/2,0.15,0.4];
t = cell(numel(t_real),1);
t_temp = 0;
t{1} = t_temp;
value_function = cell(numel(t_real),1); %store values
value_function{1} = [];
Vq_V = cell(numel(t_real),1); %store values
Vq_V{1} = [];
Vq_th = cell(numel(t_real),1); %store values
Vq_th{1} = [];
t_vf = cell(numel(t_real),1); %store values
t_vf{1} = [];
u_mat = cell(numel(t_real),1);
choice = cell(numel(t_real),1);
value1_function = cell(numel(t_real),1);
value2_function = cell(numel(t_real),1);
v2Range = [-0.15,-0.05];
u(:,3) = -0.15; %second car's velocity
value = 0;

for i = 1:numel(t_real)
    if i == 1
        [~,value1] = eval_u(traj_temp(end,:),gmatOut1,alpha_U_beta1{end+1-i},alpha_U_beta1{end+1-i},tstep_sim,value);
        [~,value2] = eval_u(traj_temp(end,:),gmatOut2,alpha_U_beta2{end+1-i},alpha_U_beta2{end+1-i},tstep_sim,value);
        
        
        if value1>value2
            [~,dVq_V,dVq_th] = eval_u(traj_temp(end,:),gmatOut1,deriv1{3}(:,:,:,:,:,end+1-i),...
                deriv1{4}(:,:,:,:,:,end+1-i),tstep_sim,value1);
            value = value1;
            choice{i} = [choice{i};1];
        else
            [~,dVq_V,dVq_th] = eval_u(traj_temp(end,:),gmatOut2,deriv2{3}(:,:,:,:,:,end+1-i),...
                deriv2{4}(:,:,:,:,:,end+1-i),tstep_sim,value2);
            value = value2;
            choice{i} = [choice{i};2];
        end
        
        t{i} = t_real(i);
        Vq_V{i} = dVq_V;
        Vq_th{i} = dVq_th;
        t_vf{i} = t_real(i);
        value_function{i} = value;
        value1_function{i} = value1;
        value2_function{i} = value2;
        traj{i} = traj_temp; %initial state
    else
        for j = 1:tstep/tstep_sim
            %compute value function value at current position
            [~,value1] = eval_u(traj_temp(end,:),gmatOut1,alpha_U_beta1{end+2-i}(:,:,:,:,:),alpha_U_beta1{end+2-i}(:,:,:,:,:),tstep_sim,value1);
            [~,value2] = eval_u(traj_temp(end,:),gmatOut2,alpha_U_beta2{end+2-i}(:,:,:,:,:),alpha_U_beta2{end+2-i}(:,:,:,:,:),tstep_sim,value2);
            
             %compute gradient of value function and control action
            if value1>value2
                [u(j,1:2),dVq_V,dVq_th] = eval_u_deriv(traj_temp(end,:),gmatOut1,deriv1{3}(:,:,:,:,:,end+2-i),deriv1{4}(:,:,:,:,:,end+2-i),...
                    deriv_R1{4}(:,:,:,:,:,end+2-i),deriv_L1{4}(:,:,:,:,:,end+2-i),tstep_sim,value1);
                value = value1;
                choice{i} = [choice{i};1];
            else
                [u(j,1:2),dVq_V,dVq_th] = eval_u_deriv(traj_temp(end,:),gmatOut2,deriv2{3}(:,:,:,:,:,end+2-i),deriv2{4}(:,:,:,:,:,end+2-i),...
                    deriv_R2{4}(:,:,:,:,:,end+2-i),deriv_L2{4}(:,:,:,:,:,end+2-i),tstep_sim,value2);
                value = value2;
                choice{i} = [choice{i};2];
            end
            

            
            %compute trajectory
            [t_temp,traj_temp] = ode45(@(t_temp,traj_temp) odefun_dubinsCar(t_temp,traj_temp,u(j,:)),...
                [t_temp(end) t_temp(end)+tstep_sim],traj_temp(end,:));
            t{i} = [t{i};t_temp(2:end)];
            
            Car2_bounds = find(traj_temp(:,5) < -0.6-v2Range(2)*tstep);
            traj_temp(Car2_bounds,5) = -0.6-v2Range(2)*tstep;
            
            %store value function gradient
            Vq_V{i} = [Vq_V{i} dVq_V];
            Vq_th{i} = [Vq_th{i} dVq_th];
            t_vf{i} = [t_vf{i} t_temp(1)];
            
            %store value function values
            value_function{i} = [value_function{i} value];
            value1_function{i} = [value1_function{i} value1];
            value2_function{i} = [value2_function{i} value2];
            
            %store trajectory
            traj{i} = [traj{i};traj_temp(2:end,:)];
            if traj_temp(end,1)>=12 || traj_temp(end,1)<=-12 || traj_temp(end,2)>=12 || traj_temp(end,2)<=-12
                break
            end
        end
    end
    
    u_mat{i} = u;
end

u_mat{1} = u_mat{1}(1,:);


save("traj.mat","traj","value_function","Vq_V","Vq_th","t_vf","t","u_mat")
%%
%load alpha_U_beta1_ori.mat
%load alpha_U_beta1.mat
load traj.mat
for i = 1:length(alpha_U_beta1)
    alpha_U_beta{i} = max(alpha_U_beta1{i},alpha_U_beta2{i});
end
%alpha_U_beta = alpha_U_beta1;
%load traj_pass3.mat

T= 12;
%t_mat = [1,11,20,24,25,26,27,39,49];
t_mat = [1,11,20,27,39,49];
%t_mat = [1,17,33,49];
y2 = zeros(numel(t_mat),1);
for i = 1:numel(t_mat)
    y2(i) = traj{t_mat(i)}(end,5);
end

V2_I = round(interpn(linspace(g.min(5),g.max(5),g.N(5)),1:g.N(5),y2));
V2_I(isnan(V2_I))=min(V2_I);
caxismin = -10;
caxismax = 2.5;


close all; figure;
set(gcf, 'Position', [100    44   936   790])

for k = 1:numel(t_mat)
    
%     [gmat,datamat] = cpp2matG(g,alpha_U_beta_d4(k));
%     [gmat2] = processGrid(gmat);
%     gmat2.dim = 4;
%     [g2d,data2d] = proj(gmat2,datamat,[0,0,1],'max');
    if k == 1
        [gmat,datamat] = cpp2matG(g,alpha_U_beta(end+1-t_mat(k)));
        [gmat2] = processGrid(gmat);
        [g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],[traj{t_mat(k)}(1,3),traj{t_mat(k)}(end,4),traj{t_mat(k)}(end,5)]);
        %[g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],'max');
        %[g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],[pi/2,0.1,0.1]);
    else
        [gmat,datamat] = cpp2matG(g,alpha_U_beta(end+2-t_mat(k)));
        [gmat2] = processGrid(gmat);
        [g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],[traj{t_mat(k)}(1,3),traj{t_mat(k)}(end,4),traj{t_mat(k)}(end,5)]);
        %[g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],'max');
        %[g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],[pi/2,0.1,0.1]);
    end
    subplot(2,3,k)
    X = linspace(g2d.min(1),g2d.max(1),g2d.N(1));
    Y = linspace(g2d.min(2),g2d.max(2),g2d.N(2));
    [~,h]=contourf(X,Y,data2d',[-100:0.1:10]);
    colorbar;
    caxis([caxismin,caxismax])
    hold on;
    [~,h2] = contour(X,Y,data2d',[0 0],'ShowText','on');
    set(h2,'LineColor','k');
    xlabel('x (m)')
    ylabel('y (m)')
    title(['\theta=' num2str(traj{t_mat(k)}(end,3)*180/pi,'%.0f') '^o, t=' num2str(((t_mat(k)-1)*T)/(length(alpha_U_beta)-1),'%.0f') 's'])
    set(h,'LineColor','none');
    axis([-0.6 0.6 -0.6 0.6])
    plot([0.2,-0.4],[-0.4,0.2],'xk','Markersize',15,'LineWidth',6);
    



    %subplot(2,2,k); hold on;
    
    max_j = t_mat(k);
    if y2(k) >-0.55
        plot(-0.2,y2(k),'ok','MarkerSize',15,'MarkerFaceColor','red');
    end

    for j = 1:max_j
        plot(traj{j}(:,1),traj{j}(:,2),'LineWidth',4)
    end
    plot(traj{max_j}(end,1),traj{max_j}(end,2),'ok','MarkerSize',15,'MarkerFaceColor','white')
    [x_arrow,y_arrow] = pol2cart(traj{max_j}(end,3),0.1);
    base_x = traj{max_j}(end,1);
    base_y = traj{max_j}(end,2);
    
    rho = 0.2;
    %rho_scale = 0.2/0.15;
    %rho = rho_scale*traj{max_j}(end,4);
    q = quiver(base_x,base_y,rho*cos(traj{max_j}(end,3)),rho*sin(traj{max_j}(end,3)));
    q.Color = 'black';
    q.LineWidth = 2;
    q.MaxHeadSize = 1;
    q.AutoScale = 'off';
    
    rho2 = -rho;
    %rho2 = rho_scale*u(k,3);
    base_y2 = traj{max_j}(end,5);
    q2 = quiver(-0.2,base_y2,0,rho2);
    q2.Color = 'black';
    q2.LineWidth = 2;
    q2.MaxHeadSize = 1;
    q2.AutoScale = 'off';
    
        ax = gca;
ax.FontSize = 15;
end

%%
figure; hold on
for i=1:length(alpha_U_beta)
    plot(t_vf{i},value_function{i},'-o')
end
title('Value function')

figure; hold on
for i = 1:length(alpha_U_beta)
    plot(t_vf{i},Vq_V{i},'-o')
    %legend_cell{i}=['t = ' num2str(t_vf{i}(1)) 's'];
end
title('dV(x dot)/dt(t,x(t)) vs t')
xlabel('t')
ylabel('dV(x dot)/dt(t,x(t))')

figure; hold on
for i = 1:length(alpha_U_beta)
    plot(t_vf{i},Vq_th{i},'-o')
    %legend_cell{i}=['t = ' num2str(t_vf{i}(1)) 's'];
end
title('dV(\theta)/dt(t,x(t)) vs t')
xlabel('t')
ylabel('dV(\theta)/dt(t,x(t))')

%%
figure; subplot(6,1,1); hold on
for i = 1:length(alpha_U_beta)
    plot(t_vf{i},u_mat{i}(:,2),'-o')
    %legend_cell{i}=['t = ' num2str(t_vf{i}(1)) 's'];
end
title('acceleration(t) vs t')
xlabel('t')
ylabel('a(t)')

subplot(6,1,2); hold on
for i = 1:length(alpha_U_beta)
    plot(t{i},traj{i}(:,4))
end
title('velocity(t) vs t')
xlabel('t')
ylabel('v(t)')

subplot(6,1,3); hold on
for i = 1:length(alpha_U_beta)
    plot(t{i},traj{i}(:,5))
end
title('\theta(t) vs t')
xlabel('t')
ylabel('\theta(t)')

subplot(6,1,4); hold on
for i = 1:length(alpha_U_beta)
    plot(t_vf{i},Vq_V{i},'-o')
    %legend_cell{i}=['t = ' num2str(t_vf{i}(1)) 's'];
end
title('dV(x dot)/dt(t,x(t)) vs t')
xlabel('t')
ylabel('dV(x dot)/dt(t,x(t))')

subplot(6,1,5); hold on
for i = 1:length(alpha_U_beta)
    plot(t_vf{i},Vq_th{i},'-o')
    %legend_cell{i}=['t = ' num2str(t_vf{i}(1)) 's'];
end
title('dV(\theta)/dt(t,x(t)) vs t')
xlabel('t')
ylabel('dV(\theta)/dt(t,x(t))')

subplot(6,1,6); hold on
for i=1:length(alpha_U_beta)
    plot(t_vf{i},value_function{i},'-o')
end
title('Value function')


%%
figure; subplot(2,1,1); hold on
for i = 1:length(alpha_U_beta)
    plot(t_vf{i},u_mat{i}(:,1),'-o')
    %legend_cell{i}=['t = ' num2str(t_vf{i}(1)) 's'];
end
title('\omega(t) vs t')
xlabel('t')
ylabel('\omega(t)')

subplot(2,1,2); hold on
for i = 1:length(alpha_U_beta)
    plot(t{i},traj{i}(:,3))
    %legend_cell{i}=['t = ' num2str(t_vf{i}(1)) 's'];
end
title('\theta(t) vs t')
xlabel('t')
ylabel('\theta(t)')

% figure; hold on
% for i = 1:12
%     plot(t_vf{i}-3,value_function{i},'LineWidth',2)
%     %legend_cell{i}=['t = ' num2str(t_vf{i}(1)) 's'];
% end
% title('V(t,x(t)) vs t')
% xlabel('t')
% ylabel('V(t,x(t))')
% %legend(legend_cell)
% run plotfixer.m

%%
% subplot(2,3,1); hold on;
%     for j = 1:max_j-1
%         plot(traj{j}(:,1),traj{j}(:,2),'k','LineWidth',2)
%     end

%%
figure; hold on;
i = length(alpha_U_beta)-3;
traj_check = traj{i}(5,:);
th_mat = linspace(0,2*pi,g.N(3)-1);
value_check = zeros(numel(th_mat),1);
for j = 1:numel(th_mat)
    traj_check(3) = th_mat(j);
    [~,value_check(j)] = eval_u(traj_check,gmatOut,alpha_U_beta{end+2-i}(:,:,:,:,:),alpha_U_beta{end+2-i}(:,:,:,:,:),tstep_sim,value);
end
plot(th_mat*180/pi,value_check,'o')

%%
figure; hold on;
i = length(alpha_U_beta1)-3;
traj_check = traj{i}(5,:);
th_mat = linspace(0,2*pi,g.N(3)-1);
dVq_th_check = zeros(numel(th_mat),1);
for j = 1:numel(th_mat)
    traj_check(3) = th_mat(j);
    [u(j,1:2),dVq_V,dVq_th_check(j)] = eval_u_deriv(traj_check,gmatOut1,deriv1{3}(:,:,:,:,:,end+2-i),deriv1{4}(:,:,:,:,:,end+2-i),...
    deriv1{4}(:,:,:,:,:,end+2-i),deriv1{4}(:,:,:,:,:,end+2-i),tstep_sim,value);
end
plot(th_mat*180/pi,dVq_th_check,'o')

