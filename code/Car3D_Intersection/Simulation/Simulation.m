clear all
load alpha_U_beta.mat
[gmat,datamat] = cpp2matG(g,alpha_U_beta);
gmatOut = processGrid(gmat,datamat(:,:,:,1));
deriv = computeGradients(gmatOut,datamat);
save('deriv.mat','deriv','gmatOut')

%%
% clear all
% close all
% load alpha_U_beta.mat
% load deriv.mat
T = 3; %total time
tstep = T/(length(alpha_U_beta)-1); %timestep between aUb frames
tstep_sim = tstep/5; %timestep between control evaluations
t_real = 0:tstep:T; %s
t_sim = 0:tstep_sim:T;
u = zeros(numel(t_real),1);
state = zeros(numel(t_sim),3);
traj = cell(numel(t_real),1);
traj_temp = [2,-7,pi/2];
traj{1} = traj_temp; %initial state

t = cell(numel(t_real),1);
t_temp = 0;
t{1} = t_temp;

for i = 1:numel(t_real)-1
    for j = 1:tstep/tstep_sim
        u(i) = eval_u(traj_temp(end,:),gmatOut,{deriv{1}(:,:,:,end+1-i);deriv{2}(:,:,:,end+1-i);deriv{3}(:,:,:,end+1-i)});
        [t_temp,traj_temp] = ode45(@(t_temp,traj_temp) odefun_dubinsCar(t_temp,traj_temp,u(i)),...
            [t_temp(end) t_temp(end)+tstep_sim],traj_temp(end,:));
        t{i} = [t{i};t_temp(2:end)];
        traj{i} = [traj{i};traj_temp(2:end,:)];
        if traj_temp(end,1)>=12 || traj_temp(end,1)<=-12 || traj_temp(end,2)>=12 || traj_temp(end,2)<=-12
            break
        end
    end
end

%%
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
    title(['y vs. x at \theta=[0,2\pi] t=' num2str((-(t_mat(k)-1)*5)/20) 's'])
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
        plot(traj{j}(:,1),traj{j}(:,2),'k','LineWidth',2)
    end
end

%%
% subplot(2,3,1); hold on;
%     for j = 1:max_j-1
%         plot(traj{j}(:,1),traj{j}(:,2),'k','LineWidth',2)
%     end
