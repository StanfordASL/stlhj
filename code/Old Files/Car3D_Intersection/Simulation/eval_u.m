function u = eval_u(state,g,deriv_t)
w_mat = -1:1; %range of allowable control
eps = 0;%0.02; %deviation of values from 0 that do not warrant a change in steering angle (u=0)

Ix = 1 + (state(1)-g.min(1))/g.dx(1);
Iy = 1 + (state(2)-g.min(2))/g.dx(2);

state(3) = wrapTo2Pi(state(3));
Ith = 1 + (state(3)-g.min(3))/g.dx(3);

Ix_L = floor(Ix);
Ix_R = ceil(Ix);
Iy_L = floor(Iy);
Iy_R = ceil(Iy);

Ith_L = floor(Ith);
Ith_R = ceil(Ith);

if Ith_R > g.N(3)
    Ith_R = 1;
end

if Ith_L > g.N(3)
    Ith_L = 1;
end

if Ix_L == Ix_R
    Ixd = 0;
else
    Ixd = (Ix-Ix_L)/(Ix_R-Ix_L);
end

if Iy_L == Iy_R
    Iyd = 0;
else
    Iyd = (Iy-Iy_L)/(Iy_R-Iy_L);
end

if Ith_R - Ith_L < 0
    Ithd = (Ith-Ith_L)/((Ith_R+g.max(3))-Ith_L);
elseif Ith_L == Ith_R
    Ithd = 0;
else
    Ithd = (Ith-Ith_L)/(Ith_R-Ith_L);
end

%Trilinear interpolation
deriv_th = deriv_t{3};
c000 = deriv_th(Ix_L,Iy_L,Ith_L);
c100 = deriv_th(Ix_R,Iy_L,Ith_L);
c001 = deriv_th(Ix_L,Iy_L,Ith_R);
c101 = deriv_th(Ix_R,Iy_L,Ith_R);
c010 = deriv_th(Ix_L,Iy_R,Ith_L);
c110 = deriv_th(Ix_R,Iy_R,Ith_L);
c011 = deriv_th(Ix_L,Iy_R,Ith_R);
c111 = deriv_th(Ix_R,Iy_R,Ith_R);

c00 = c000*(1-Ixd) + c100*Ixd;
c01 = c001*(1-Ixd) + c101*Ixd;
c10 = c010*(1-Ixd) + c110*Ixd;
c11 = c011*(1-Ixd) + c111*Ixd;

c0 = c00*(1-Iyd)+c10*Iyd;
c1 = c01*(1-Iyd)+c11*Iyd;

c = c0*(1-Ithd)+c1*Ithd;

if c>=-eps && c<=eps
    u = 0;
elseif c<0
    u = min(w_mat);
elseif c>0
    u = max(w_mat);
end





z