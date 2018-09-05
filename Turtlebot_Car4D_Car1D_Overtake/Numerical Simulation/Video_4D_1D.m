%%
clear all
% close all
%load alpha_U_beta.mat
load overtake_output.mat
load deriv.mat
load derivLR.mat
T=25;
tstep = T/(length(alpha_U_beta)-1); %timestep between aUb frames
tstep_sim = 0.125/2; %timestep between control evaluations
t_real = 0:tstep:T; 
u = zeros(tstep/tstep_sim,3);

traj = cell(numel(t_real),1);
%traj_temp = [-0.1,0.,pi/2,0,0.8];
traj_temp = [.2,-0.55,90*pi/180,0.0,-0.55];
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

u(:,3) = 0.04; %second car's velocity
value = 0.04;

for i = 1:numel(t_real)
    if i == 1
        [~,value] = eval_u(traj_temp(end,:),gmatOut,alpha_U_beta{end+1-i},alpha_U_beta{end+1-i},tstep_sim,value);
       
        [~,dVq_V,dVq_th] = eval_u(traj_temp(end,:),gmatOut,deriv{3}(:,:,:,:,:,i+1),...
            deriv{4}(:,:,:,:,:,end+1-i),tstep_sim,value);
        
        t{i} = t_real(i);
        Vq_V{i} = dVq_V;
        Vq_th{i} = dVq_th;
        t_vf{i} = t_real(i);
        value_function{i} = value;
        traj{i} = traj_temp; %initial state
    else
        for j = 1:tstep/tstep_sim
            %compute value function value at current position
            [~,value] = eval_u(traj_temp(end,:),gmatOut,alpha_U_beta{end+2-i}(:,:,:,:,:),alpha_U_beta{end+2-i}(:,:,:,:,:),tstep_sim,value);
            
            %compute gradient of value function and control action
            [u(j,1:2),dVq_V,dVq_th] = eval_u_deriv(traj_temp(end,:),gmatOut,deriv,deriv_R,deriv_L,tstep_sim,value,i+1);
            
            %compute trajectory
            [t_temp,traj_temp] = ode45(@(t_temp,traj_temp) odefun_dubinsCar(t_temp,traj_temp,u(j,:)),...
                [t_temp(end) t_temp(end)+tstep_sim],traj_temp(end,:));
            t{i} = [t{i};t_temp(2:end)];
            
            Car2_bounds = find(traj_temp(:,5) >= 0.7);
            traj_temp(Car2_bounds,5) = 0.7;
            
            %store value function gradient
            Vq_V{i} = [Vq_V{i} dVq_V];
            Vq_th{i} = [Vq_th{i} dVq_th];
            t_vf{i} = [t_vf{i} t_temp(1)];
            
            %store value function values
            value_function{i} = [value_function{i} value];
            
            %store trajectory
            traj{i} = [traj{i};traj_temp(2:end,:)];
%             if traj_temp(end,1)>=12 || traj_temp(end,1)<=-12 || traj_temp(end,2)>=12 || traj_temp(end,2)<=-12
%                 break
%             end
        end
    end
    
    u_mat{i} = u;
end

u_mat{1} = u_mat{1}(1,:);


save("traj.mat","traj","value_function","Vq_V","Vq_th","t_vf","t","u_mat")

%load overtake_output.mat
% %load alpha_U_beta.mat
% %load traj_pass3.mat
% load traj.mat
T = 25;
%T= 25;%t_mat = linspace(1,81,9);
%t_mat = [1 14 26 39 51 64 76 89 101];
t_mat = ceil(linspace(1,101,4));
%t_mat = [1,17,33,49]
%t_mat = [1,3,5,7,9,11,13];
y2 = zeros(numel(t_mat),1);
for i = 1:numel(t_mat)
    y2(i) = traj{t_mat(i)}(end,5);
end

V2_I = round(interpn(linspace(g.min(5),g.max(5),g.N(5)),1:g.N(5),y2));
V2_I(isnan(V2_I))=min(V2_I);
caxismin = -3;
caxismax = 3.;
%%
%
%%
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
[~,h]=contourf(X,Y,data2d',[-3:0.1:3]);
colorbar;
caxis([caxismin,caxismax])
%[~,h2] = contour(X,Y,data2d',[0 0],'ShowText','on');
%set(h2,'LineColor','k');
xlabel('x (m)')
ylabel('y (m)')
%title(['\theta=' num2str(traj{t_mat(k)}(end,3)*180/pi,'%.0f') '^o, t=' num2str(((t_mat(k)-1)*T)/(length(alpha_U_beta)-1),'%.0f') 's'])
set(h,'LineColor','none');

axis equal
axis([-0.4 0.4 -0.6 0.6])
T = 25; %total time
tstep = T/(length(alpha_U_beta)-1); %timestep between aUb frames
tstep_sim = 0.125/2; %timestep between control evaluations
t_real = 0:tstep:T;
set(gcf, 'Position', [100    44   936   790])
ax = gca;
ax.FontSize = 20;
camroll(90);
for k = 1:numel(t_real)
    for i = 1:4
        if k == 1 
            I_end = 1;
        else
            I_end = i*floor(size(traj{k},1))/4 - 1*floor(size(traj{k},1))/4 + 1;
        end
        
        if k == 1
            
            [gmat,datamat] = cpp2matG(g,alpha_U_beta(end+1-k));
            [gmat2] = processGrid(gmat);
            [g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],[traj{k}(end,3),traj{k}(end,4),traj{k}(end,5)]);
            
        else
            if traj{k}(I_end,4)>0.11
                traj{k}(I_end,4) = 0.11;
            elseif traj{k}(I_end,4)<0
                traj{k}(I_end,4)=0;
            end
            [gmat,datamat] = cpp2matG(g,alpha_U_beta(end+2-k));
            [gmat2] = processGrid(gmat);
            [g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],[traj{k}(I_end,3),traj{k}(I_end,4),traj{k}(I_end,5)]);
%             if k==2 && i==2
%                 [g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],[traj{k}(I_end-40,3),traj{k}(I_end-40,4),traj{k}(I_end-40,5)]);
%             end
                
            %[g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],'max');
        end
        
        [~,h]=contourf(X,Y,data2d',[-100:0.1:10]);
        %[~,h2] = contour(X,Y,data2d',[0 0],'ShowText','on');
        set(h,'LineColor','none');
        delete( findobj(gca, 'type', 'line') );
%         time_display=((k-1)*T)/(length(alpha_U_beta)-1)+0.125/2*(i-1);
%         k_display = k;
%         i_display = i;
%         vars_display = [time_display,k_display,i_display,traj{k}(I_end,3),traj{k}(I_end,4),traj{k}(I_end,5)]
        
        title([num2str(((k-1)*T)/(length(alpha_U_beta)-1)+0.125/2*(i-1),'%.2f') 's'])
        
        %title(['\theta=' num2str(traj{k}(i,3)*180/pi,'%.0f') '^o, t=' num2str(((t_mat(k)-1)*T)/(length(alpha_U_beta)-1),'%.0f') 's'])
        
       % plot([0.2,-0.4],[-0.4,0.2],'xk','Markersize',15,'LineWidth',6);
        if traj{k}(I_end,5)<0.9
            h5=plot(-0.2,traj{k}(I_end,5),'ok','MarkerSize',27,'MarkerFaceColor','red','LineWidth',7);
        end
        
        %plot(traj{j}(:,1),traj{j}(:,2),'LineWidth',4)
        
        h4=plot(traj{k}(I_end,1),traj{k}(I_end,2),'ok','MarkerSize',27,'MarkerFaceColor','white','LineWidth',7);
        [x_arrow,y_arrow] = pol2cart(traj{k}(I_end,3),0.1);
        base_x = traj{k}(I_end,1);
        base_y = traj{k}(I_end,2);
        
        h6 = plot(0.2,0.,'ok','MarkerSize',27,'MarkerFaceColor','green','LineWidth',7);
        
        %rho = 0.2;
        rho_scale = 0.1;
        rho = 0.2;
        q = quiver(base_x,base_y,rho*cos(traj{k}(I_end,3)),rho*sin(traj{k}(I_end,3)));
        q.Color = 'black';
        q.LineWidth = 2;
        q.MaxHeadSize = 1;
        q.AutoScale = 'off';
        
        
        
        if traj{k}(I_end,5)<0.9
            %rho2 = -rho;
            rho2 = rho_scale*0.15/0.15 + 0.1;
            base_y2 = traj{k}(I_end,5);
            q2 = quiver(-0.2,base_y2,0,rho2);
            q2.Color = 'black';
            q2.LineWidth = 2;
            q2.MaxHeadSize = 1;
            q2.AutoScale = 'off';
        end
        
        q3 = quiver(0.2,0,0,0.2);
        q3.Color = 'black';
        q3.LineWidth = 2;
        q3.MaxHeadSize = 1;
        q3.AutoScale = 'off';
    
        
        set(gca,'xcolor','w','ycolor','w','xtick',[],'ytick',[])
        frame=getframe(gcf); 
        writeVideo(v,frame);
        if k == numel(t_real)
            break;
        end
        
    end
end
close(v)

% %%
% figure; hold on;
% i = 46;
% traj_check = traj{i-1}(end,:);
% v_mat = linspace(g.min(4),g.max(4),g.N(4));
% dVq_V_check = zeros(numel(v_mat),1);
% for j = 1:numel(v_mat)
%     traj_check(4) = v_mat(j);
%     [u(j,1:2),dVq_V_check(j),dVq_th_check] = eval_u_deriv(traj_check,gmatOut1,deriv2{3}(:,:,:,:,:,end+2-i),deriv2{4}(:,:,:,:,:,end+2-i),...
%                     deriv_R2{4}(:,:,:,:,:,end+2-i),deriv_L2{4}(:,:,:,:,:,end+2-i),tstep_sim,value1)
% end
% plot(v_mat,dVq_V_check,'o')
% 
% %%
% figure; hold on;
% i = 46;
% traj_check = traj{i-1}(end,:);
% v_mat = linspace(g.min(4),g.max(4),g.N(4));
% dVq_V_check = zeros(numel(v_mat),1);
% for j = 1:numel(v_mat)
%     traj_check(4) = v_mat(j);
%     [u(j,1:2),dVq_V_check(j),dVq_th_check] = eval_u_deriv(traj_check,gmatOut1,alpha_U_beta2{end+2-i}(:,:,:,:,:),alpha_U_beta2{end+2-i}(:,:,:,:,:),...
%                     alpha_U_beta2{end+2-i}(:,:,:,:,:),alpha_U_beta2{end+2-i}(:,:,:,:,:),tstep_sim,value1);
% end
% plot(v_mat,dVq_V_check,'o')