clear all
load alpha_U_beta.mat
[gmat,datamat] = cpp2matG(g,alpha_U_beta);
gmatOut = processGrid(gmat,datamat(:,:,:,:,:,1));
deriv = computeGradients(gmatOut,datamat);
save('deriv.mat','deriv','gmatOut')

%%
clear all
close all
load alpha_U_beta.mat
load deriv.mat
T = 5; %total time
tstep = T/(length(alpha_U_beta)-1); %timestep between aUb frames
tstep_sim = tstep/10; %timestep between control evaluations
t_real = 0:tstep:T; %s
u = zeros(numel(t_real)-1,1);
t = cell(numel(t_real)-1,1);
traj = cell(numel(t_real)-1,1);
traj_temp = [2,-5,pi/2,0,2];
traj{1} = traj_temp; %initial state

t = cell(numel(t_real),1);
t_temp = 0;
t{1} = t_temp;
value_function = cell(numel(t_real)-1,1); %store values
value_function{1} = [];
dvalue_function = cell(numel(t_real)-1,1); %store values
dvalue_function{1} = [];
t_vf = cell(numel(t_real)-1,1); %store values
t_vf{1} = [];

for i = 1:numel(t_real)-1
    for j = 1:tstep/tstep_sim
        %compute value function value at current position
        [~,value] = eval_u(traj_temp(end,:),gmatOut,{alpha_U_beta{end+1-i}(:,:,:,:,:);alpha_U_beta{end+1-i}(:,:,:,:,:);alpha_U_beta{end+1-i}(:,:,:)});
        
        %compute gradient of value function and control action
        [u(i),dvalue] = eval_u(traj_temp(end,:),gmatOut,{deriv{1}(:,:,:,:,:,end+1-i);deriv{2}(:,:,:,:,:,end+1-i);deriv{3}(:,:,:,:,:,end+1-i)});
        
        %compute trajectory
        [t_temp,traj_temp] = ode45(@(t_temp,traj_temp) odefun_dubinsCar(t_temp,traj_temp,u(i)),...
            [t_temp(end) t_temp(end)+tstep_sim],traj_temp(end,:));
        t{i} = [t{i};t_temp(2:end)];
        
        %store value function gradient
        dvalue_function{i} = [dvalue_function{i} dvalue];
        t_vf{i} = [t_vf{i} t_temp(1)];
        
        %store value function values
        value_function{i} = [value_function{i} value];
        
        %store trajectory
        traj{i} = [traj{i};traj_temp(2:end,:)];
        if traj_temp(end,1)>=12 || traj_temp(end,1)<=-12 || traj_temp(end,2)>=12 || traj_temp(end,2)<=-12
            break
        end
    end
end

%%
close all
figure;
load alpha_U_beta.mat
t_mat = [1:3:10,12,13];

for k = 1:numel(t_mat)
    [gmat,datamat] = cpp2matG(g,alpha_U_beta(t_mat(k)));
    [gmat2] = processGrid(gmat);
    [g2d,data2d] = proj(gmat2,datamat,[0,0,1],'max');
    subplot(2,3,7-k)
    X = linspace(g2d.min(1),g2d.max(1),g2d.N(1));
    Y = linspace(g2d.min(2),g2d.max(2),g2d.N(2));
    [~,h]=contourf(X,Y,data2d',[-100:0.1:10]);
    colorbar;
    hold on;
    [~,h2] = contour(X,Y,data2d',[0 0],'ShowText','on');
    set(h2,'LineColor','k');
    xlabel('x (m)')
    ylabel('y (m)')
    title(['y vs. x at \theta=[0,2\pi] t=' num2str((-(t_mat(k)-1)*3)/12) 's'])
    set(h,'LineColor','none');
    axis([-10 10 -10 10])
    
end

for k = 2:numel(t_mat)
    subplot(2,3,k); hold on;
    if k>= i
        max_j = i;
    else
        max_j = t_mat(k);
    end
    
    for j = 1:max_j-1
        plot(traj{j}(:,1),traj{j}(:,2),'LineWidth',4)
    end
end
%%
figure; hold on
for i = 1:12
    plot(t_vf{i}-3,dvalue_function{i},'LineWidth',2)
    %legend_cell{i}=['t = ' num2str(t_vf{i}(1)) 's'];
end
title('dV/dt(t,x(t)) vs t')
xlabel('t')
ylabel('dV/dt(t,x(t))')

figure; hold on
for i = 1:12
    plot(t_vf{i}-3,value_function{i},'LineWidth',2)
    %legend_cell{i}=['t = ' num2str(t_vf{i}(1)) 's'];
end
title('V(t,x(t)) vs t')
xlabel('t')
ylabel('V(t,x(t))')
%legend(legend_cell)
run plotfixer.m

%%
% subplot(2,3,1); hold on;
%     for j = 1:max_j-1
%         plot(traj{j}(:,1),traj{j}(:,2),'k','LineWidth',2)
%     end
