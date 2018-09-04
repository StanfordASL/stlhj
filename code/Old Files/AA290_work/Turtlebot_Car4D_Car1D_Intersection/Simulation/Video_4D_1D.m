%%
clear all
% close all
load alpha_U_beta.mat
load deriv.mat
load derivLR.mat
T = 12; %total time
tstep = T/(length(alpha_U_beta)-1); %timestep between aUb frames
tstep_sim = 0.125/2; %timestep between control evaluations
t_real = 0:tstep:T;
u = zeros(tstep/tstep_sim,3);

traj = cell(numel(t_real),1);
%traj_temp = [0.2,-0.6,pi/2,0.15,0.5]; %no pass
traj_temp = [-0.05,-0.45,pi/2,0.15,0.4];
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

u(:,3) = -0.15; %second car's velocity
value = 0;

for I_end = 1:numel(t_real)
    if I_end == 1
        [~,value] = eval_u(traj_temp(end,:),gmatOut,alpha_U_beta{end+1-I_end},alpha_U_beta{end+1-I_end},tstep_sim,value);
        
        [~,dVq_V,dVq_th] = eval_u(traj_temp(end,:),gmatOut,deriv{3}(:,:,:,:,:,end+1-I_end),...
            deriv{4}(:,:,:,:,:,end+1-I_end),tstep_sim,value);
        
        t{I_end} = t_real(I_end);
        Vq_V{I_end} = dVq_V;
        Vq_th{I_end} = dVq_th;
        t_vf{I_end} = t_real(I_end);
        value_function{I_end} = value;
        traj{I_end} = traj_temp; %initial state
    else
        for j = 1:tstep/tstep_sim
            %compute value function value at current position
            [~,value] = eval_u(traj_temp(end,:),gmatOut,alpha_U_beta{end+2-I_end}(:,:,:,:,:),alpha_U_beta{end+2-I_end}(:,:,:,:,:),tstep_sim,value);
            
            %compute gradient of value function and control action
            [u(j,1:2),dVq_V,dVq_th] = eval_u_deriv(traj_temp(end,:),gmatOut,deriv{3}(:,:,:,:,:,end+2-I_end),deriv{4}(:,:,:,:,:,end+2-I_end),...
                deriv_R{4}(:,:,:,:,:,end+2-I_end),deriv_L{4}(:,:,:,:,:,end+2-I_end),tstep_sim,value);
            
            %compute trajectory
            [t_temp,traj_temp] = ode45(@(t_temp,traj_temp) odefun_dubinsCar(t_temp,traj_temp,u(j,:)),...
                [t_temp(end) t_temp(end)+tstep_sim],traj_temp(end,:));
            t{I_end} = [t{I_end};t_temp(2:end)];
            
            Car2_bounds = find(traj_temp(:,5)<=-0.6);
            traj_temp(Car2_bounds,5) = -0.6;
            
            %store value function gradient
            Vq_V{I_end} = [Vq_V{I_end} dVq_V];
            Vq_th{I_end} = [Vq_th{I_end} dVq_th];
            t_vf{I_end} = [t_vf{I_end} t_temp(1)];
            
            %store value function values
            value_function{I_end} = [value_function{I_end} value];
            
            %store trajectory
            traj{I_end} = [traj{I_end};traj_temp(2:end,:)];
            if traj_temp(end,1)>=12 || traj_temp(end,1)<=-12 || traj_temp(end,2)>=12 || traj_temp(end,2)<=-12
                break
            end
        end
    end
    
    u_mat{I_end} = u;
end

u_mat{1} = u_mat{1}(1,:);

save("traj.mat","traj")
%%
clear all
load alpha_U_beta.mat
load traj.mat

%
%t_mat = [1,11,20,30,39,49];
%%
caxismin = -10;
caxismax = 2.5;
%%
%
close all; h3 = figure; hold on;


v = VideoWriter('Test.avi');
v.FrameRate = 16;
open(v)
k = 1;
[gmat,datamat] = cpp2matG(g,alpha_U_beta(end+1-k));
[gmat2] = processGrid(gmat);
[g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],[traj{k}(end,3),traj{k}(end,4),traj{k}(end,5)]);
X = linspace(g2d.min(1),g2d.max(1),g2d.N(1));
Y = linspace(g2d.min(2),g2d.max(2),g2d.N(2));
[~,h]=contourf(X,Y,data2d',[-100:0.1:10]);
colorbar;
caxis([caxismin,caxismax])
%[~,h2] = contour(X,Y,data2d',[0 0],'ShowText','on');
%set(h2,'LineColor','k');
xlabel('x (m)')
ylabel('y (m)')
%title(['\theta=' num2str(traj{t_mat(k)}(end,3)*180/pi,'%.0f') '^o, t=' num2str(((t_mat(k)-1)*T)/(length(alpha_U_beta)-1),'%.0f') 's'])
set(h,'LineColor','none');
axis([-0.6 0.6 -0.6 0.6])
T = 12; %total time
tstep = T/(length(alpha_U_beta)-1); %timestep between aUb frames
tstep_sim = 0.125/2; %timestep between control evaluations
t_real = 0:tstep:T;
set(gcf, 'Position', [100    44   936   790])
ax = gca;
ax.FontSize = 20;

for k = 1:numel(t_real)
    for i = 1:4
        if k == 1
            I_end = 1;
        else
        I_end = i*floor(size(traj{k},1))/4;
        end
        
        if k == 1
            
            [gmat,datamat] = cpp2matG(g,alpha_U_beta(end+1-k));
            [gmat2] = processGrid(gmat);
            [g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],[traj{k}(end,3),traj{k}(end,4),traj{k}(end,5)]);
            
        else
            [gmat,datamat] = cpp2matG(g,alpha_U_beta(end+2-k));
            [gmat2] = processGrid(gmat);
            [g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],[traj{k}(I_end,3),traj{k}(I_end,4),traj{k}(I_end,5)]);
            %[g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],'max');
            
        end
        
        [~,h]=contourf(X,Y,data2d',[-100:0.1:10]);
        %[~,h2] = contour(X,Y,data2d',[0 0],'ShowText','on');
        set(h,'LineColor','none');
        delete( findobj(gca, 'type', 'line') );
        
        
        
        title([num2str(((k-1)*T)/(length(alpha_U_beta)-1)+0.125/2*(i-1),'%.2f') 's'])
        
        %title(['\theta=' num2str(traj{k}(i,3)*180/pi,'%.0f') '^o, t=' num2str(((t_mat(k)-1)*T)/(length(alpha_U_beta)-1),'%.0f') 's'])
        
        plot([0.2,-0.4],[-0.4,0.2],'xk','Markersize',15,'LineWidth',6);
        if traj{k}(I_end,5)>-0.59
            h5=plot(-0.2,traj{k}(I_end,5),'ok','MarkerSize',30,'MarkerFaceColor','red','LineWidth',8);
        end
        
        %plot(traj{j}(:,1),traj{j}(:,2),'LineWidth',4)
        
        h4=plot(traj{k}(I_end,1),traj{k}(I_end,2),'ok','MarkerSize',30,'MarkerFaceColor','white','LineWidth',8);
        [x_arrow,y_arrow] = pol2cart(traj{k}(I_end,3),0.1);
        base_x = traj{k}(I_end,1);
        base_y = traj{k}(I_end,2);
        
        %rho = 0.2;
        rho_scale = 0.1;
        rho = rho_scale*traj{k}(I_end,4)/0.15 + 0.1;
        q = quiver(base_x,base_y,rho*cos(traj{k}(I_end,3)),rho*sin(traj{k}(I_end,3)));
        q.Color = 'black';
        q.LineWidth = 2;
        q.MaxHeadSize = 1;
        q.AutoScale = 'off';
        
        
        
        if traj{k}(I_end,5)>-0.59
            %rho2 = -rho;
            rho2 = -rho_scale*0.15/0.15 - 0.1;
            base_y2 = traj{k}(I_end,5);
            q2 = quiver(-0.2,base_y2,0,rho2);
            q2.Color = 'black';
            q2.LineWidth = 2;
            q2.MaxHeadSize = 1;
            q2.AutoScale = 'off';
            
        end
        frame=getframe(gcf); 
        writeVideo(v,frame);
        if k == numel(t_real)
            break;
        end
    end
    
end
close(v)
