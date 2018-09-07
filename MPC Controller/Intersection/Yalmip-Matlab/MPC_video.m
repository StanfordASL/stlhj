close all; h3 = figure; hold on;


v = VideoWriter('Test.avi');
v.FrameRate = 10;
open(v)

xlabel('x (m)')
ylabel('y (m)')
%title(['\theta=' num2str(traj{t_mat(k)}(end,3)*180/pi,'%.0f') '^o, t=' num2str(((t_mat(k)-1)*T)/(length(alpha_U_beta)-1),'%.0f') 's'])
%set(h,'LineColor','none');
axis([-0.6 0.6 -0.6 0.6])

set(gcf, 'Position', [100    44   936   790])
ax = gca;
ax.FontSize = 20;

[X,Y] = ndgrid(xd,yd);
Z = zeros(numel(xd),numel(yd));

[~,h] = contourf(X,Y,Z,[-100:.1:10]); 
%colormap pink
colorbar;
caxis([-12,2.5])
set(h,'LineColor','none');

[~,h2] = contour(X,Y,Z,[0,0],'k');

rho_scale = 0.1;
rho = 0.2;
q = quiver(0,0,0,0);
q.Color = 'black';
q.LineWidth = 2;
q.MaxHeadSize = 1;
q.AutoScale = 'off';

rho2 = -rho_scale*0.15/0.15 - 0.1;
base_y2 = traj(1,5);
q2 = quiver(0,0,0,0);
q2.Color = 'black';
q2.LineWidth = 2;
q2.MaxHeadSize = 1;
q2.AutoScale = 'off';
V1 = griddedInterpolant({xd,yd,thd,vd,y2d,td},alpha_U_beta1_Ext); 
V2 = griddedInterpolant({xd,yd,thd,vd,y2d,td},alpha_U_beta2_Ext); 
%t_iter = floor(numel(tau)/120);

for i = 1:numel(tau)
 
        delete( findobj(gca, 'type', 'line') );
        title([num2str(tau(i),'%.2f') 's'])
        
        if MaxValueFunction(i) == 1
            Z = V1({xd,yd,traj(i,3),traj(i,4),traj(i,5),traj(i,6)});
        else
            Z = V2({xd,yd,traj(i,3),traj(i,4),traj(i,5),traj(i,6)});
        end
        %[~,h] = contourf(X,Y,Z); 
        set(h,'ZData',Z);
        set(h2,'ZData',Z);
        plot([0.2,-0.4],[-0.4,0.2],'xk','Markersize',15,'LineWidth',6);
        if traj(i,5)>-0.55
            h5=plot(-0.2,traj(i,5),'ok','MarkerSize',30,'MarkerFaceColor','red','LineWidth',8);
        end
        
        h4=plot(traj(i,1),traj(i,2),'ok','MarkerSize',30,'MarkerFaceColor','white','LineWidth',8);

        base_x = traj(i,1);
        base_y = traj(i,2);       
        set(q,'xdata',base_x,'ydata',base_y,'udata',rho*cos(traj(i,3)),'vdata',rho*sin(traj(i,3)))
        
        if traj(i,5)>-0.55
            base_y2 = traj(i,5);
            set(q2,'xdata',-0.2,'ydata',base_y2,'udata',0,'vdata',rho2)
        else
            set(q2,'xdata',-0.2,'ydata',base_y2,'udata',0,'vdata',0)
        end
        
        set(gca,'xcolor','w','ycolor','w','xtick',[],'ytick',[])
        frame=getframe(gcf); 
        writeVideo(v,frame);

end
close(v)