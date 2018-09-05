clear all
load alpha.mat
%close all
[gmat,datamat] = cpp2matG(g,{data});
[gmat2] = processGrid(gmat);
% %%
% % 3D visualization of Target Set Values
% figure
% visSetIm(gmat2,datamat,'r',0); %x,y,th

% 2D Projection 
%close all
th_mat = [0:2*pi/8:2*pi];
max_caxis = max(datamat(:));
min_caxis = min(datamat(:));
%th = 5*pi/4;
V = 0.1;
y2 = -0.6;
figure;
for i = 1:numel(th_mat)-1
subplot(2,4,i);
th = th_mat(i);
[g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],[th_mat(i),V,0.4]);
%[g2d,data2d] = proj(gmat2,datamat,[0,0,1,1,1],'max');
%visSetIm(g2d,data2d,'r');
% Contour plot (y vs.x)
%subplot13,3,i)
X = linspace(g2d.min(1),g2d.max(1),g2d.N(1));
Y = linspace(g2d.min(2),g2d.max(2),g2d.N(2));
[~,h]=contourf(X,Y,data2d',[-100:0.1:10]);
colorbar;
caxismin = 0.3;
caxismax = 2;
%caxis([caxismin caxismax]);
hold on;
[~,h2] = contour(X,Y,data2d',[0 0],'ShowText','on');
set(h2,'LineColor','k');
xlabel('x (m)')
ylabel('y (m)')
title(['x-y plane at \theta=' num2str(th*180/pi) '^o'])
%title('alpha')
%title(['Max Projection on x-y Plane'])

set(h,'LineColor','none');
%axis([-0.6 0.6 -0.6 0.6])
%set(gcf, 'Position', [100    44   936   790])
set(gcf,'Position',[113 -66 1732 549])
%ax = gca;
%ax.FontSize = 20;
end
%% Contour plot (theta vs. x) Road_test.mat - Single Lane
close all
X = linspace(g2d.min(1),g2d.max(1),g2d.N(1));
Y = linspace(g2d.min(2),g2d.max(2),g2d.N(2));
contour(X,Y,data2d',[-7:0.5:0.5],'ShowText','on')
xlabel('x')
ylabel('\theta')
hold on
h1 = plot([-4,4],[pi/2,pi/2],'--k')
h2 = plot([2,2],[g2d.min(2),g2d.max(2)],'k')
plot([-2,-2],[g2d.min(2),g2d.max(2)],'k')
legend([h1,h2],'\theta offset','road boundary')

%% Contour plot (y vs. x) Road_test.mat - Single Lane
close all
X = linspace(g2d.min(1),g2d.max(1),g2d.N(1));
Y = linspace(g2d.min(2),g2d.max(2),g2d.N(2));
levels =  -6:0.5:0.5;
contour(X,Y,data2d',levels,'ShowText','on')
xlabel('x')
ylabel('y')
hold on
%h1 = plot([-4,4],[pi/2,pi/2],'--k')
h2 = plot([2,2],[g2d.min(2),g2d.max(2)],'k')
plot([-2,-2],[g2d.min(2),g2d.max(2)],'k')
legend(h2,'road boundary')