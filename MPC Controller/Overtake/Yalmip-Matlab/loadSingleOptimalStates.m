function single_OptStates = loadSingleOptimalStates(states,F,th_mat,v_mat)
%%loadOptimalStates
OptStates = [];
%   load the optimal heading and speed states from the value function

%[xq,yq,thq,vq,y2q] = ndgrid(states(1),states(2),th_mat,states(4),states(5));
%J_v = ndgrid(states(1),states(2),states(3),v_mat,states(5));
J_th = F({states(1),states(2),th_mat,states(4),states(5),states(6)});
J_v = F({states(1),states(2),states(3),v_mat,states(5),states(6)});

[~,I_th] = max(J_th);
[~,I_v] = max(J_v);

single_OptStates(1) = th_mat(I_th);
single_OptStates(2) = v_mat(I_v);

end

