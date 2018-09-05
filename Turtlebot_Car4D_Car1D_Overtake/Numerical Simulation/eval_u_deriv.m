function [u,Vq_V,Vq_th] = eval_u_deriv(state,g,deriv_C,deriv_R,deriv_L,t_step,value,i)
u = zeros(2,1);

deriv_th = deriv_C{3}(:,:,:,:,:,end+2-i);
deriv_V = deriv_C{4}(:,:,:,:,:,end+2-i);
deriv_th_R = deriv_R{3}(:,:,:,:,:,end+2-i);
deriv_V_R = deriv_R{4}(:,:,:,:,:,end+2-i);
deriv_th_L = deriv_L{3}(:,:,:,:,:,end+2-i);
deriv_V_L = deriv_L{4}(:,:,:,:,:,end+2-i);

vrange = [0,0.11]; %range of acceptable velocities
thrange = [40*pi/180,140*pi/180];
w_mat = [-1.,1.]; %range of allowable turning rate
a_mat = [-0.2,0.2]; %range of allowable acceleration
v_mat = linspace(g.min(4),g.max(4),g.N(4));
th_mat = linspace(g.min(3),g.max(3),g.N(3));
v_thresh_min = min(v_mat(v_mat>vrange(1)));
v_thresh_max = max(v_mat(v_mat<vrange(2)));

dth = (g.max(3)-g.min(3))/g.N(3);
th_thresh_min = thrange(1)+dth;
th_thresh_max = thrange(2)-dth;
th_thresh_min2 = pi/2+dth;
th_thresh_max2 = pi/2-dth;

th_eps = 0;%0.02; %deviation of values from 0 that do not warrant a change in steering angle (u=0)
V_eps = 0;

%Linear interpolation

[xd,yd,thd,Vd,y2d] = ndgrid(linspace(g.min(1),g.max(1),g.N(1)),linspace(g.min(2),g.max(2),g.N(2)),...
    linspace(g.min(3),g.max(3),g.N(3)),linspace(g.min(4),g.max(4),g.N(4)),linspace(g.min(5),g.max(5),g.N(5)));

[xq,yq,thq,Vq,y2q] = ndgrid(state(1),state(2),state(3),state(4),state(5));

if Vq>=vrange(1) && Vq<=v_thresh_min
    Vq_V = interpn(xd,yd,thd,Vd,y2d,deriv_V_R,xq,yq,thq,Vq,y2q);
elseif Vq<=vrange(2) && Vq>=v_thresh_max
    Vq_V = interpn(xd,yd,thd,Vd,y2d,deriv_V_L,xq,yq,thq,Vq,y2q);
else
    Vq_V = interpn(xd,yd,thd,Vd,y2d,deriv_V,xq,yq,thq,Vq,y2q);
end

if thq>=thrange(1) && thq<=th_thresh_min
    Vq_th = interpn(xd,yd,thd,Vd,y2d,deriv_th_R,xq,yq,thq,Vq,y2q);
elseif thq<=thrange(2) && thq>=th_thresh_max
    Vq_th = interpn(xd,yd,thd,Vd,y2d,deriv_th_L,xq,yq,thq,Vq,y2q);
elseif thq>=pi/2 && thq<=th_thresh_min2
    Vq_th = interpn(xd,yd,thd,Vd,y2d,deriv_th_R,xq,yq,thq,Vq,y2q);
elseif thq<=pi/2 && thq>=th_thresh_max2
    Vq_th = interpn(xd,yd,thd,Vd,y2d,deriv_th_L,xq,yq,thq,Vq,y2q);
else
    Vq_th = interpn(xd,yd,thd,Vd,y2d,deriv_th,xq,yq,thq,Vq,y2q);
end


% if Vq_th>=-th_eps && Vq_th<=th_eps
%     u(1) = 0;
if Vq_th<=0
    if state(3)+min(w_mat)*t_step <= g.min(3) || abs(Vq_th) < 1e-5 %|| value <= val_threshold
        u(1) = 0;
    else 
         u(1) = min(w_mat);
    end
elseif Vq_th>=0
    if state(3)+max(w_mat)*t_step >= g.max(3)  %|| value <= val_threshold
        u(1) = 0;
    else
         u(1) = max(w_mat);
    end
end

% if Vq_V>=-V_eps && Vq_V<=V_eps
%     u(2) = 0;
if Vq_V<=0
    if state(4)+min(a_mat)*t_step <= g.min(4)
        u(2) = 0;
    else
        u(2) = min(a_mat);
    end
elseif Vq_V>=0
    if state(4)+max(a_mat)*t_step >= g.max(4) %|| %value <= val_threshold
        u(2) = 0;
    else
        u(2) = max(a_mat);
    end
end


% if value <= val_threshold
%     u(1:2) = 0;
% end










