%clear all
clear all
%load alpha_U_beta2.mat
load alpha.mat
close all
%%
% 3D visualization of Target Set Values
% figure
% visSetIm(gmat2,datamat,'r',0); %x,y,th


%%
% 2D Projection
close all
angle_mat = [0:pi/4:(2*pi-pi/4)];
%angle_mat = [pi];
t_mat = [1:4:20,21];

for j = 1:numel(angle_mat)
    figure;
    for i = 1:numel(t_mat)
        [gmat,datamat] = cpp2matG(g,alpha_U_beta(t_mat(i)));
        [gmat2] = processGrid(gmat);
        [g2d,data2d] = proj(gmat2,datamat,[0,0,1],angle_mat(j));
        %visSetIm(g2d,data2d,'r',[0]);
        % Contour plot (y vs.x)
        subplot(2,3,i)
        X = linspace(g2d.min(1),g2d.max(1),g2d.N(1));
        Y = linspace(g2d.min(2),g2d.max(2),g2d.N(2));
        [~,h]=contourf(X,Y,data2d',[-100:0.1:10]);
        colorbar;
        hold on;
        [~,h2] = contour(X,Y,data2d',[0 0],'ShowText','on');
        set(h2,'LineColor','k');
        xlabel('x (m)')
        ylabel('y (m)')
        title(['y vs. x at \theta=' num2str(angle_mat(j)*180/pi) ' t=' num2str((t_mat(i)-1)*5/20) 's'])
        set(h,'LineColor','none');
    end
end

%%
% 2D Projection
close all
%angle_mat = [0:pi/4:(2*pi-pi/4)];
angle_mat = [36:12:144]/180*pi;
%angle_mat = [pi];
%t_mat = [1:4:20,21];
%t_mat = [1:2:9,13];
%t_mat = [1,5,9,15,20,21,22,23,25];
%t_mat = [1,11,21,31,41,51,61,80,81];
%t_mat = [1,6,11,16,21,26,31,36,41];
% t_mat = ceil(linspace(1,29,9));
% t_mat(4) = 9;
t_mat = 9;
u_v2 = 0.05; %m/s
y2_0 = 0.1;%
figure;hold on
for j = 1:numel(angle_mat)
    %figure;
    for i = 1:numel(t_mat)
        %[gmat,datamat] = cpp2matG(g,always_alpha(length(always_alpha)+1-t_mat(i)));
        [gmat,datamat] = cpp2matG(g,event_beta(length(event_beta)+1-t_mat(i)));
        [gmat2] = processGrid(gmat);
        y2 = y2_0 + (((t_mat(i)-1)*5)/20)*u_v2;
        y2 = -0.5;
        [g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],[angle_mat(j),0.05,y2]);
        %[g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],'max');
        %visSetIm(g2d,data2d,'r',[0]);
        % Contour plot (y vs.x)
        subplot(2,5,j)
        X = linspace(g2d.min(1),g2d.max(1),g2d.N(1));
        Y = linspace(g2d.min(2),g2d.max(2),g2d.N(2));
%         [~,h]=contourf(X,Y,data2d',[-100:0.1:100]);
        [~,h]=contourf(X,Y,data2d');
        colorbar;
        %caxis([-10 2.5])
        hold on;
        [~,h2] = contour(X,Y,data2d',[0 0],'ShowText','on');
        set(h2,'LineColor','k');
        xlabel('x (m)')
        ylabel('y (m)')
        title(['y vs. x at \theta=' num2str(angle_mat(j)*180/pi) ' t=' num2str(((t_mat(i)-1)*5)/20) 's'])
        set(h,'LineColor','none');
        axis equal
    end
end

%% Plot value vs. velocity
figure; hold on;
vel = [-0.15:0.01:0.2];
for j = 1:numel(vel)
    for i = numel(t_mat)
%         [gmat,datamat] = cpp2matG(g,always_alpha(length(always_alpha)+1-t_mat(i)));
        [gmat,datamat] = cpp2matG(g,event_beta(end+1-t_mat(i)));
        [gmat2] = processGrid(gmat);
        [~,value1] = eval_u([0.2,0.5,pi/2,vel(j),1],gmat2,event_beta{end+1-t_mat(i)}(:,:,:,:,:),event_beta{end+1-t_mat(i)}(:,:,:,:,:),1,1);
        plot(vel(j),value1,'o');
        xlabel('value')
        ylabel('velocity (m/s)')

    end
end
%% Plot value vs. y2
figure; hold on;
y2 = linspace(g.min(5),g.max(5),g.N(5));
for j = 1:numel(y2)
    for i = numel(t_mat)
        [gmat,datamat] = cpp2matG(g,always_alpha(end+1-t_mat(i)));
        %[gmat,datamat] = cpp2matG(g,event_beta(end+1-t_mat(i)));
        [gmat2] = processGrid(gmat);
        %[~,value1] = eval_u([0.2,1.5,pi/2,0.,y2(j)],gmat2,event_beta{end+1-t_mat(i)}(:,:,:,:,:),event_beta{end+1-t_mat(i)}(:,:,:,:,:),1,1);
        [~,value1] = eval_u([0.2,1.5,pi/2,0.,y2(j)],gmat2,always_alpha{end+1-t_mat(i)}(:,:,:,:,:),always_alpha{end+1-t_mat(i)}(:,:,:,:,:),1,1);
        plot(y2(j),value1,'o');
        xlabel('value')
        ylabel('y2 (m)')

    end
end

%%
% 2D Projection
close all; clear all
load overtake_output.mat
%%
close all
%angle_mat = [0:pi/4:(2*pi-pi/4)];
%angle_mat = [pi];
angle_mat = linspace(22.5/180*pi,157.5/180*pi,11);
%t_mat = [1:4:20,21];
%t_mat = [1:2:9,13];
%t_mat = [1,5,9,15,20,21,22,23,25];
%t_mat = [1,11,21,31,41,51,61,71,81];
%t_mat = [1,5,9,13,17,21,25,29,33];%11,16,21,26,31,36,41];
%t_mat = [1,9,16,24,31,39,46,54,61];
%t_mat = [1,6,11,16,21,26,31,36,41];
%t_mat = linspace(1,21,5);
t_mat = 101;
u_v2 = 0.; %m/s
y2_0 = 0.8;
figure;
for j = 1:numel(angle_mat)
    for i = 1:numel(t_mat)
        %[gmat,datamat] = cpp2matG(g,always_alpha(length(always_alpha)+1-t_mat(i)));
        %[gmat,datamat] = cpp2matG(g,event_beta(length(event_beta)+1-t_mat(i)));
        [gmat,datamat] = cpp2matG(g,alpha_U_beta(length(alpha_U_beta)+1-t_mat(i)));
        [gmat2] = processGrid(gmat);
        y2 = y2_0 + (((t_mat(i)-1)*5)/20)*u_v2;
        %y2 = -0.;
        if y2 > 0.8
            y2  = 0.8;
        end
        [g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],[angle_mat(j),0.05,y2]);
        %[g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],'max');
        %visSetIm(g2d,data2d,'r',[0]);
        % Contour plot (y vs.x)
        subplot(2,6,j)
        X = linspace(g2d.min(1),g2d.max(1),g2d.N(1));
        Y = linspace(g2d.min(2),g2d.max(2),g2d.N(2));
        [~,h]=contourf(X,Y,data2d',[0:0.01:2.5]);
        %[~,h]=contourf(X,Y,data2d');
        colorbar;
        caxis([0 2.5])
        hold on;
        [~,h2] = contour(X,Y,data2d',[0 0],'ShowText','on');
        set(h2,'LineColor','k');
        if y2 < 0.8
            plot(-0.15,y2,'ok','MarkerSize',10,'MarkerFaceColor','red');
        end
        xlabel('x (m)')
        ylabel('y (m)')
        title(['y vs. x at \theta=' num2str(90) ' t=' num2str(((t_mat(i)-1)*5)/20) 's'])
        set(h,'LineColor','none');
        axis equal
        %axis([-0.3 0.3 -0.8 0.8]);
        
    end
end
%% mini_sim
% 2D Projection
close all
angle_mat = [0:pi/4:(2*pi-pi/4)];
%angle_mat = [pi];
t_mat = [1:21];

for j = 1%:numel(angle_mat)
    figure; hold on;
    for i = 1:length(t_mat)
        [gmat,datamat] = cpp2matG(g,event_beta(t_mat(i)));
        [gmat2] = processGrid(gmat);
        [g2d,data2d] = proj(gmat2,datamat,[0,0,1],'max');
        %visSetIm(g2d,data2d,'r',[0]);
        % Contour plot (y vs.x)
        %subplot(2,3,i)
        X = linspace(g2d.min(1),g2d.max(1),g2d.N(1));
        Y = linspace(g2d.min(2),g2d.max(2),g2d.N(2));
        [~,h]=contourf(X,Y,data2d',[-100:0.1:10]);
        colorbar;
        [~,h2] = contour(X,Y,data2d',[0 0],'ShowText','on');
        set(h2,'LineColor','k');
        xlabel('x (m)')
        ylabel('y (m)')
        title(['y vs. x at \theta=' num2str(angle_mat(j)*180/pi) ' t=' num2str(((t_mat(i)-1)*5)/20) 's'])
        set(h,'LineColor','none');
        pause(0.001);
    end
end

%%
% 2D Projection
close all
angle_mat = [0:pi/4:(2*pi-pi/4)];
%angle_mat = [pi];
%t_mat = [1:4:20,21];
t_mat = [1,11,21,31,41,51,61,71,81];
y2_0 = -0.6;
v2 =0 ;
for j = 1%:numel(angle_mat)
    figure;
    for i = 1:numel(t_mat)
        [gmat,datamat] = cpp2matG(g,event_beta(t_mat(i)));
        [gmat2] = processGrid(gmat);
        
        y2 = y2_0 + v2*(6-(t_mat(i)-1)*0.5);
        [g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],[pi/2,0.05,y2]);
        
        %visSetIm(g2d,data2d,'r',[0]);
        % Contour plot (y vs.x)
        subplot(3,3,i)
        X = linspace(g2d.min(1),g2d.max(1),g2d.N(1));
        Y = linspace(g2d.min(2),g2d.max(2),g2d.N(2));
        [~,h]=contourf(X,Y,data2d',[-100:0.1:10]);
        colorbar;
        %caxis([0 1.8])
        hold on;
        [~,h2] = contour(X,Y,data2d',[0 0],'ShowText','on');
        plot(-1.5,y2,'or','MarkerFaceColor','r');
        set(h2,'LineColor','k');
        xlabel('x (m)')
        ylabel('y (m)')
        title(['y vs. x at \theta=' num2str(angle_mat(j)*180/pi) ' t=' num2str(((t_mat(i)-1)*5)/10) 's'])
        set(h,'LineColor','none');
    end
end

%%
