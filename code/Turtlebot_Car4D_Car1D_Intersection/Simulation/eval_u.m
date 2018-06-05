function [u,Vq_V,Vq_th] = eval_u(state,g,deriv_t,t_step,value)
u = zeros(2,1);
val_threshold = 0;
w_mat = [-0.4,0.4]; %range of allowable turning rate
a_mat = [-0.2,0.2]; %range of allowable acceleration
th_eps = 0;%0.02; %deviation of values from 0 that do not warrant a change in steering angle (u=0)
V_eps = 0;

%Trilinear interpolation
deriv_th = deriv_t{3};
deriv_V = deriv_t{4};

% [xd,yd,thd,Vd,y2d] = ndgrid(linspace(g.min(1),g.max(1),g.N(1)),linspace(g.min(2),g.max(2),g.N(2)),...
%     linspace(g.min(3),g.max(3),g.N(3)),linspace(g.min(4),g.max(4),g.N(4)),linspace(g.min(5),g.max(5),g.N(5)));
% 
% [xq,yq,thq,Vq,y2q] = ndgrid(state(1),state(2),state(3),state(4),state(5));
% 
% Vq_th = interpn(xd,yd,thd,Vd,y2d,deriv_th,xq,yq,thq,Vq,y2q);
% 
% Vq_V = interpn(xd,yd,thd,Vd,y2d,deriv_V,xq,yq,thq,Vq,y2q);

[~,I_1] = min(abs(linspace(g.min(1),g.max(1),g.N(1))-state(1)));
[~,I_2] = min(abs(linspace(g.min(2),g.max(2),g.N(2))-state(2)));
[~,I_3] = min(abs(linspace(g.min(3),g.max(3),g.N(3))-state(3)));
[~,I_4] = min(abs(linspace(g.min(4),g.max(4),g.N(4))-state(4)));
[~,I_5] = min(abs(linspace(g.min(5),g.max(5),g.N(5))-state(5)));

Vq_V = deriv_V(I_1,I_2,I_3,I_4,I_5);
Vq_th = deriv_th(I_1,I_2,I_3,I_4,I_5);


if Vq_th>=-th_eps && Vq_th<=th_eps
    u(1) = 0;
elseif Vq_th<=0
    if state(3)+min(w_mat)*t_step <= g.min(3) %|| value <= val_threshold
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

if Vq_V>=-V_eps && Vq_V<=V_eps
    u(2) = 0;
elseif Vq_V<=0
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










