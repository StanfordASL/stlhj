%clear all
clear all
load alpha_U_beta2.mat
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
angle_mat = [0:pi/4:(2*pi-pi/4)];
%angle_mat = [pi];
%t_mat = [1:4:20,21];
t_mat = [1:2:9,13];
for j = 1%:numel(angle_mat)
    figure;
    for i = 1:numel(t_mat)
        [gmat,datamat] = cpp2matG(g,alpha_U_beta2(t_mat(i)));
        [gmat2] = processGrid(gmat);
        [g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],'max');
        %visSetIm(g2d,data2d,'r',[0]);
        % Contour plot (y vs.x)
        subplot(2,3,i)
        X = linspace(g2d.min(1),g2d.max(1),g2d.N(1));
        Y = linspace(g2d.min(2),g2d.max(2),g2d.N(2));
        [~,h]=contourf(X,Y,data2d',[-100:0.1:10]);
        colorbar;
        %caxis([0 1.8])
        hold on;
        [~,h2] = contour(X,Y,data2d',[0 0],'ShowText','on');
        set(h2,'LineColor','k');
        xlabel('x (m)')
        ylabel('y (m)')
        title(['y vs. x at \theta=' num2str(angle_mat(j)*180/pi) ' t=' num2str(((t_mat(i)-1)*5)/20) 's'])
        set(h,'LineColor','none');
    end
end

%%
% 2D Projection
close all
angle_mat = [0:pi/4:(2*pi-pi/4)];
%angle_mat = [pi];
t_mat = [1:21];

for j = 1%:numel(angle_mat)
    figure; hold on;
    for i = 1:numel(t_mat)
        [gmat,datamat] = cpp2matG(g,alpha_U_beta(t_mat(i)));
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
t_mat = [1:2:9,11];
y2_0 = -0.5;
v2 =0 ;
for j = 1%:numel(angle_mat)
    figure;
    for i = 1:numel(t_mat)
        [gmat,datamat] = cpp2matG(g,alpha_U_beta(t_mat(i)));
        [gmat2] = processGrid(gmat);
        
        y2 = y2_0 + v2*(6-(t_mat(i)-1)*0.5);
        [g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],[pi/2,0.2,y2_0]);
        
        %visSetIm(g2d,data2d,'r',[0]);
        % Contour plot (y vs.x)
        subplot(2,3,i)
        X = linspace(g2d.min(1),g2d.max(1),g2d.N(1));
        Y = linspace(g2d.min(2),g2d.max(2),g2d.N(2));
        [~,h]=contourf(X,Y,data2d',[-100:0.1:10]);
        colorbar;
        %caxis([0 1.8])
        hold on;
        [~,h2] = contour(X,Y,data2d',[0 0],'ShowText','on');
        plot(-2,y2,'or','MarkerFaceColor','r');
        set(h2,'LineColor','k');
        xlabel('x (m)')
        ylabel('y (m)')
        title(['y vs. x at \theta=' num2str(angle_mat(j)*180/pi) ' t=' num2str(((t_mat(i)-1)*5)/10) 's'])
        set(h,'LineColor','none');
    end
end