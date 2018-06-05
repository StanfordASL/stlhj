clear all
load alpha_U_beta.mat
[gmat,datamat] = cpp2matG(g,alpha_U_beta);
gmatOut = processGrid(gmat,datamat(:,:,:,:,:,1));
deriv = computeGradients(gmatOut,datamat);
save('deriv.mat','deriv','gmatOut')

%%
clear all
% close all
load alpha_U_beta.mat
load deriv.mat
T = 12; %total time
tstep = T/(length(alpha_U_beta)-1); %timestep between aUb frames
tstep_sim = 0.125; %timestep between control evaluations
t_real = 0:tstep:T; %s
u = zeros(tstep/tstep_sim,3);

traj = cell(numel(t_real),1);
traj_temp = [0.125,-0.25,pi/2,0.,0.5];

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

u(:,3) = -0.05; %second car's velocity
value = 0;

for i = 1:numel(t_real)
    if i == 1
        [~,value] = eval_u(traj_temp(end,:),gmatOut,{alpha_U_beta{end+1-i}(:,:,:,:,:);alpha_U_beta{end+1-i}(:,:,:,:,:);...
            alpha_U_beta{end+1-i}(:,:,:,:,:);alpha_U_beta{end+1-i}(:,:,:,:,:);alpha_U_beta{end+1-i}(:,:,:,:,:)},tstep_sim,value);
       
        [~,dVq_V,dVq_th] = eval_u(traj_temp(end,:),gmatOut,{deriv{1}(:,:,:,:,:,end+1-i);deriv{2}(:,:,:,:,:,end+1-i);...
            deriv{3}(:,:,:,:,:,end+1-i);deriv{4}(:,:,:,:,:,end+1-i);deriv{5}(:,:,:,:,:,end+1-i)},tstep_sim,value);
        
        t{i} = t_real(i);
        Vq_V{i} = dVq_V;
        Vq_th{i} = dVq_th;
        t_vf{i} = t_real(i);
        value_function{i} = value;
        traj{i} = traj_temp; %initial state
    else
        for j = 1:tstep/tstep_sim
            %compute value function value at current position
            [~,value] = eval_u(traj_temp(end,:),gmatOut,{alpha_U_beta{end+2-i}(:,:,:,:,:);alpha_U_beta{end+2-i}(:,:,:,:,:);...
                alpha_U_beta{end+2-i}(:,:,:,:,:);alpha_U_beta{end+2-i}(:,:,:,:,:);alpha_U_beta{end+2-i}(:,:,:,:,:)},tstep_sim,value);
            
            %compute gradient of value function and control action
            [u(j,1:2),dVq_V,dVq_th] = eval_u(traj_temp(end,:),gmatOut,{deriv{1}(:,:,:,:,:,end+2-i);deriv{2}(:,:,:,:,:,end+2-i);...
                deriv{3}(:,:,:,:,:,end+2-i);deriv{4}(:,:,:,:,:,end+2-i);deriv{5}(:,:,:,:,:,end+2-i)},tstep_sim,value);
            
            %compute trajectory
            [t_temp,traj_temp] = ode45(@(t_temp,traj_temp) odefun_dubinsCar(t_temp,traj_temp,u(j,:)),...
                [t_temp(end) t_temp(end)+tstep_sim],traj_temp(end,:));
            t{i} = [t{i};t_temp(2:end)];
            
            Car2_bounds = find(traj_temp(:,5)<=g.min(5));
            traj_temp(Car2_bounds,5) = g.min(5);
            
            %store value function gradient
            Vq_V{i} = [Vq_V{i} dVq_V];
            Vq_th{i} = [Vq_th{i} dVq_th];
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
    
    u_mat{i} = u;
end

u_mat{1} = u_mat{1}(1,:);

save("traj.mat","traj")

%load alpha_U_beta.mat
load traj.mat
t_mat = [1,3,5,8,11,13];
alpha_U_beta_d4 = cell(length(t_mat),1);
y2 = zeros(numel(t_mat),1);
for i = 1:numel(t_mat)
    y2(i) = traj{t_mat(i)}(end,5);
end

V2_I = round(interpn(linspace(g.min(5),g.max(5),g.N(5)),1:g.N(5),y2));
V2_I(isnan(V2_I))=min(V2_I);

close all; figure;
for i = 1:length(t_mat)
    alpha_U_beta_d4{i} = alpha_U_beta{end+1-t_mat(i)}(:,:,:,:,V2_I(i)); %take out car2's dim
end
for k = 1:numel(t_mat)
    %[gmat,datamat] = cpp2matG(g,alpha_U_beta_d4(k));
    %g[gmat2] = processGrid(gmat);
    %gmat2.dim = 4;
    %[g2d,data2d] = proj(gmat2,datamat,[0,0,1,1],'max');
    if k == 1
        [gmat,datamat] = cpp2matG(g,alpha_U_beta(end+1-t_mat(k)));
        [gmat2] = processGrid(gmat);
        [g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],[traj{t_mat(k)}(end,3),traj{t_mat(k)}(end,4),traj{t_mat(k)}(end,5)]);

    else
        [gmat,datamat] = cpp2matG(g,alpha_U_beta(end+2-t_mat(k)));
        [gmat2] = processGrid(gmat);
        [g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],[traj{t_mat(k)}(end,3),traj{t_mat(k)}(end,4),traj{t_mat(k)}(end,5)]);

    end
    subplot(2,3,k)
    X = linspace(g2d.min(1),g2d.max(1),g2d.N(1));
    Y = linspace(g2d.min(2),g2d.max(2),g2d.N(2));
    [~,h]=contourf(X,Y,data2d',[-100:0.1:10]);
    colorbar;
    caxis([-10,1])
    hold on;
    [~,h2] = contour(X,Y,data2d',[0 0],'ShowText','on');
    set(h2,'LineColor','k');
    xlabel('x (m)')
    ylabel('y (m)')
    title(['\theta=' num2str(traj{t_mat(k)}(end,3)*180/pi,'%.0f') '^o, t=' num2str(((t_mat(k)-1)*T)/(length(alpha_U_beta)-1),'%.0f') 's'])
    set(h,'LineColor','none');
    axis([-0.5 0.5 -0.5 0.5])
    
end

for k = 1:numel(t_mat)
    subplot(2,3,k); hold on;
    
    max_j = t_mat(k);
    plot(-0.125,y2(k),'ok','MarkerSize',15,'MarkerFaceColor','red');
    

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
    q2 = quiver(-0.125,base_y2,0,rho2);
    q2.Color = 'black';
    q2.LineWidth = 2;
    q2.MaxHeadSize = 1;
    q2.AutoScale = 'off';
end
%%
figure; hold on
for i=1:13
    plot(t_vf{i},value_function{i},'-o')
end
title('Value function')

figure; hold on
for i = 1:13
    plot(t_vf{i},Vq_V{i},'-o')
    %legend_cell{i}=['t = ' num2str(t_vf{i}(1)) 's'];
end
title('dV(x dot)/dt(t,x(t)) vs t')
xlabel('t')
ylabel('dV(x dot)/dt(t,x(t))')

figure; hold on
for i = 1:13
    plot(t_vf{i},u_mat{i}(:,2),'-o')
    %legend_cell{i}=['t = ' num2str(t_vf{i}(1)) 's'];
end
title('u(t) vs t')
xlabel('t')
ylabel('u(t)')

figure; hold on
for i = 1:13
    plot(t_vf{i},Vq_th{i},'-o')
    %legend_cell{i}=['t = ' num2str(t_vf{i}(1)) 's'];
end
title('dV(\theta)/dt(t,x(t)) vs t')
xlabel('t')
ylabel('dV(\theta)/dt(t,x(t))')

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
