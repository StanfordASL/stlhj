%% Optimization with NLP
clear all;
close all; clc


%% Include Dependencies into Path
% comment this if you already have YALMIP in your path somewhere else...
addpath(genpath('..\AA290\MPC-STLHJ\MPC_Intersection'));
% addpath(genpath('..\..\MATLAB\YALMIP-master'));

%% NLP Settings
settings                        = sdpsettings;
settings.solver                 = 'ipopt';
% settings.ipopt.print_level      = 5;    % default = 5
% settings.ipopt.tol              = 1e-3; % default = 1e-8
% settings.ipopt.dual_inf_tol     = 1;    % default = 1
% settings.ipopt.constr_viol_tol  = 1e-2; % default = 1e-4
% settings.ipopt.compl_inf_tol    = 1e-4; % default = 1e-4
% settings.ipopt.acceptable_tol   = 1e-6; % default = 1e-6
% settings.ipopt.acceptable_iter  = 10;   % default = 15
%     % tol info: https://www.coin-or.org/Ipopt/documentation/node42.html#SECTION000112010000000000000
settings.verbose                = 0;
% settings.debug                  = 0;
% settings.usex0                  = 0;

%% Load Lookup Table
load overtake_output_Ext.mat

%% Define Parameters
% load Boundary Values and Opt Params
BVs          = loadSolveParameters(g);   % init and final vals found here
fn           = fieldnames(BVs);
for i = 1:length(fn)
   p.(fn{i}) = BVs.(fn{i});             % load struct BVs into p
end

xd = linspace(g.min(1),g.max(1),g.N(1));
yd = linspace(g.min(2),g.max(2),g.N(2));
thd = linspace(g.min(3),g.max(3),g.N(3));
vd = linspace(g.min(4),g.max(4),g.N(4));
y2d = linspace(g.min(5),g.max(5),g.N(5));
td = 0:0.25:27;

V1 = griddedInterpolant({xd,yd,thd,vd,y2d,td},overtake_output_Ext);

%% Optimization Object
integration = 0; %Heun's
integration_iters = 2;
[objective, constraints, variables, x, y, th, v, y2, t, omega, accel, v2_input, th_bar, v_bar] = Dynamics_and_Constraints(p,integration,integration_iters);

%% Solve
% diagnostics = optimize([ constraints
%     x.variable(1)       == p.states(1)/x.const 
%     y.variable(1)       == p.states(2)/y.const 
%     th.variable(1)      == p.states(3)/th.const 
%     v.variable(1)       == p.states(4)/v.const
%     y2.variable(1)      == p.states(5)/y2.const 
%     t.variable(1)       == p.states(6)/t.const 
%     ],objective,settings);

% parameters_in = {x.physical(1),y.physical(1),th.physical(1),v.physical(1),...
%     y2.physical(1),t.physical(1),omega.physical(1),accel.physical(1),...
%     th_bar.physical,v_bar.physical};
parameters_in = {[x.variable(1),y.variable(1),th.variable(1),v.variable(1),...
    y2.variable(1),t.variable(1)],[omega.variable(1),accel.variable(1)],v2_input.variable,...
    th_bar.physical,v_bar.physical};
solutions_out = {x.physical,y.physical,th.physical,v.physical,y2.physical,t.physical,omega.physical,accel.physical};
controller = optimizer(constraints,objective,settings,parameters_in,solutions_out);
%%
controls = zeros(p.N+1,2);
controls(1,:) = [0,0];
states = zeros(p.N+1,6);

%[x_0,y_0,th_0,v_0,y2_0,t_0]
%states(1,:) = [.2,-0.55,90*pi/180,0.0,-0.55,0]; %Pass
states(1,:) = [0.2,-0.55,pi/2,0.,-0.55,0]; %NoPass

traj_temp = states(1,:);
v2 = 0.04;
%% Outputs
% load Optimal States (th_bar, v_bar)
tau = 0;
t_temp = 0;
traj = traj_temp;

if integration == 0
    v2_mat_MPC = repmat(v2,1,p.N*integration_iters);
    v2_mat_real = repmat(v2,1,p.N*integration_iters);
elseif integration == 1
    v2_mat_MPC = repmat(v2,1,p.N+1);
    v2_mat_real = repmat(v2,1,p.N*integration_iters);
end

OptStates_Set = zeros(25/p.dt,2);
MaxValueFunction = ones(25/p.dt+1,1);
for i = 1:25/p.dt
    [OptStates, MaxValueFunction(i+1)] = loadOptimalStates(states(i,:),V1,V1,p,thd,vd,v2,integration,integration_iters);
    OptStates_Set(i+1,:) = [OptStates.th_bar(1),OptStates.v_bar(1)];
    [sol,error] = controller({states(i,:),controls(i,:),v2_mat_MPC,OptStates.th_bar,OptStates.v_bar});
    controls(i+1,:) = [sol{7}(1+integration_iters),sol{8}(1+integration_iters)];
    
    %integrate states
    %states(i+1,:)= euler_integration_states(states(i,:),controls(i+1,:),p.dt,v2);
    [t_temp,traj_temp] = ode45(@(t_temp,traj_temp) odefun_dubinsCar(t_temp,traj_temp,[controls(i+1,:),v2_mat_real]),...
        [0 p.dt],traj_temp(end,:));
    tau = [tau;tau(end)+t_temp(end)];
    traj = [traj;traj_temp(end,:)];
    states(i+1,:) = traj_temp(end,:);
   
    if states(i+1,5) >= 0.7
        states(i+1,5) = 0.7;
        traj_temp(end,5) = 0.7;
    end
end

%% Plots
close all
figure; hold on;
plot(traj(:,1),traj(:,2));
axis equal
axis([-0.4 0.4 -0.6 0.6])

figure; hold on;
plot(states(:,1),states(:,2),'-o')
axis equal
axis([-0.4 0.4 -0.6 0.6])

figure; hold on;
plot(states(:,6),states(:,3),'-o')
plot(states(:,6),OptStates_Set(:,1),'-or')
title('Heading,\theta(t)')
legend('MPC','V_max')

figure; hold on;
plot(states(:,6),states(:,4),'-o')
plot(states(:,6),OptStates_Set(:,2),'-or')
title('Speed,v(t)')
legend('MPC','V_max')

figure; hold on;
plot(states(:,6),controls(:,1),'-o')
plot(states(:,6),controls(:,2),'-o')
legend('\omega','acceleration')

% close all
% figure; hold on;
% plot(sol{1},sol{2},'-ob'); 
% plot(OptStates.x_bar,OptStates.y_bar,'-xb')
% legend('MPC','Bang-bang')
% axis([-1 1 -1 1])
% 
% figure; hold on; 
% plot(sol{6}(1:end-1),sol{8},'-o'); 
% plot(sol{6}(1:end-1),sol{7},'-o'); 
% legend('acceleration','omega')
% 
% 
% figure; hold on; 
% plot(sol{6},sol{4},'-or'); 
% plot(sol{6},OptStates.v_bar,'-xr'); 
% ylim([g.min(4) g.max(4)])
% ylabel('velocity (m/s)')
% 
% yyaxis right
% plot(sol{6},sol{3},'-ob'); 
% plot(sol{6},OptStates.th_bar,'-xb'); 
% ylim([g.min(3) g.max(3)])
% ylabel('theta (rad)')
% legend('velocity','velocity (ideal)','theta','theta (ideal)')

%% cleanup extra variables
clearvars fn copynames i integration message;





