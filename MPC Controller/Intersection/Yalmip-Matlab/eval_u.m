function value = eval_u(state,g,value_function)
%Linear interpolation

[xd,yd,thd,Vd,y2d] = ndgrid(linspace(g.min(1),g.max(1),g.N(1)),linspace(g.min(2),g.max(2),g.N(2)),...
    linspace(g.min(3),g.max(3),g.N(3)),linspace(g.min(4),g.max(4),g.N(4)),linspace(g.min(5),g.max(5),g.N(5)));

[xq,yq,thq,Vq,y2q] = ndgrid(state(1),state(2),state(3),state(4),state(5));

value = interpn(xd,yd,thd,Vd,y2d,value_function,xq,yq,thq,Vq,y2q);









