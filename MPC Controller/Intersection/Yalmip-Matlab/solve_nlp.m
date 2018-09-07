function sol = solve_nlp(N,F,initial_state,state_min,state_max,ctrl_min,ctrl_max,state_with_time_grid,Value_function)

import casadi.*

% Start with an empty NLP
w={};
w0 = [];
lbw = [];
ubw = [];
J = 0;
g={};
lbg = [];
ubg = [];

% "Lift" initial conditions
Xk = MX.sym('X0', 6);
w = {w{:}, Xk};
lbw = [lbw; initial_state];
ubw = [ubw; initial_state];
w0 = [w0; initial_state];

% Formulate the NLP
for k=0:N-1
    % New NLP variable for the control
    Uk = MX.sym(['U_' num2str(k)],2);
    w = {w{:}, Uk};
    lbw = [lbw; ctrl_min];
    ubw = [ubw; ctrl_max];
    w0 = [w0;  0; 0];

    % Integrate till the end of the interval
    Fk = F('x0', Xk, 'p', Uk);
    Xk_end = Fk.xf;
    
    find_max_V = [];
    v_range = linspace(-0.075,0.2250,21);
    for v_ind = 1:21
        find_max_V = [find_max_V,Xk_end(1:3),v_range(v_ind),Xk_end(5:6)];
    end
    
    %J=J+Fk.qf;
    
    LUT = interpolant('LUT','linear',state_with_time_grid,Value_function(:));
    max(find_max
    %J = J +  (10*Uk(1))^2 + (Uk(2)*10)^2; %+ (Xk_end(7)-Xk(7))^2 + (Xk_end(8)-Xk(8))^2; 
    J = J - max(LUT(Xk_end(1:3)));% +  (Uk(1))^2 + (Uk(2))^2;
    
    % New NLP variable for state at end of interval
    Xk = MX.sym(['X_' num2str(k+1)], 6);
    w = [w, {Xk}];
    lbw = [lbw; state_min; 0];
    ubw = [ubw; state_max; Inf];
    w0 = [w0; initial_state];

    % Add equality constraint
    g = [g, {Xk_end-Xk}];
    lbg = [lbg; 0; 0; 0; 0; 0; 0];
    ubg = [ubg; 0; 0; 0; 0; 0; 0];
end

% Create an NLP solver
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));

% opt.print_time = 0;
% opt.verbose_init=0;
% opt.ipopt.print_level = 0;
solver = nlpsol('solver', 'ipopt', prob);


% Solve the NLP
sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,...
            'lbg', lbg, 'ubg', ubg);
%w_opt = full(sol.x);