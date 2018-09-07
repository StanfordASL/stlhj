function [objective, constraints, variables, x, y, th, v, y2, t, omega, accel, v2, th_bar, v_bar] = Dynamics_and_Constraints(p,integration,Iter_per_control_interval)

% Copy paramteres
copynames = fieldnames(p);
for i = 1:length(copynames)
    eval(strcat(copynames{i},' = p.(copynames{',num2str(i),'});'))
end

N = N*Iter_per_control_interval;
dt = dt/Iter_per_control_interval;
%% Initialize Variables
% INPUT VARIABLES
if integration == 0
    accel   = mdlvar(N,1,'input');
    omega   = mdlvar(N,1,'input');
    v2      = mdlvar(N,1,'input');
elseif integration == 1
    accel   = mdlvar(N+1,1,'input');
    omega   = mdlvar(N+1,1,'input');
    v2      = mdlvar(N+1,1,'input');
end
    

% STATE VARIABLES
x        = mdlvar(N+1,1,'state');
y       = mdlvar(N+1,1,'state');
th      = mdlvar(N+1,1,'state');
v       = mdlvar(N+1,1,'state');
y2      = mdlvar(N+1,1,'state');
t       = mdlvar(N+1,1,'state');

% COST VARIABLES
th_bar = mdlvar(N+1,1,'tracking');
v_bar = mdlvar(N+1,1,'tracking');

%% Inizialize Constraints
constraints = [];
 
% Differential Equations, Euler Integration
        
if integration == 0
    constraints = [ constraints
        ( diff(x.variable)  == (v.physical(1:N).*cos(th.physical(1:N))).*dt/x.const ): 'x Position'
        ( diff(y.variable)  == (v.physical(1:N).*sin(th.physical(1:N))).*dt/y.const): 'y Position'
        ( diff(th.variable) ==  omega.physical(1:N).*dt/th.const ): 'Heading'
        ( diff(v.variable)  ==  accel.physical(1:N).*dt/v.const ): 'Velocity'
        ( diff(y2.variable)  ==  v2.physical(1:N).*dt/y2.const): 'Velocity'
        ( diff(t.variable)  ==  dt/t.const ): 'Time'
                   
                   ];
elseif integration == 1
   constraints = [ constraints
        ( diff(x.variable)  == (v.physical(1:N).*cos(th.physical(1:N))).*dt/x.const/2 ...
                                + (v.physical(2:N+1).*cos(th.physical(2:N+1))).*dt/x.const/2 ): 'x Position'
        ( diff(y.variable)  == (v.physical(1:N).*sin(th.physical(1:N))).*dt/y.const/2 ...
                                + (v.physical(2:N+1).*sin(th.physical(2:N+1))).*dt/y.const/2): 'y Position'
        ( diff(th.variable) ==  omega.physical(1:N).*dt/th.const/2 ...
                                + omega.physical(2:N+1).*dt/th.const/2): 'Heading'
        ( diff(v.variable)  ==  accel.physical(1:N).*dt/v.const/2 ...
                                + accel.physical(2:N+1).*dt/v.const/2): 'Velocity'
        ( diff(y2.variable)  ==  v2.physical(1:N).*dt/y2.const/2 ...
                                + v2.physical(2:N+1).*dt/y2.const/2): 'Velocity'
        ( diff(t.variable)  ==  dt/t.const ): 'Time'

                  ];
end

% State Constraints
constraints = [ constraints
    th.variable    >=  thMin/th.const
    th.variable    <=  thMax/th.const
    v.variable     >=  vMin/v.const
    v.variable     <=  vMax/v.const
    x.variable     >=  xMin/x.const
    x.variable     <=  xMax/x.const
    y.variable     >=  yMin/y.const
    y.variable     <=  yMax/y.const
    ];

% Initial Conditions
% constraints = [ constraints
%     x.variable(1)       == states(1)/x.const 
%     y.variable(1)       == states(2)/y.const 
%     th.variable(1)      == states(3)/th.const 
%     v.variable(1)       == states(4)/v.const
%     y2.variable(1)      == states(5)/y2.const 
%     t.variable(1)       == states(6)/t.const 
%     ];

% constraints = [ constraints
%     x.physical(1)       == states(1)/x.const 
%     y.physical(1)       == states(2)/y.const 
%     th.physical(1)      == states(3)/th.const 
%     v.physical(1)       == states(4)/v.const
%     y2.physical(1)      == states(5)/y2.const 
%     t.physical(1)       == states(6)/t.const 
%     ];

% Input Constraints
constraints = [ constraints
    omega.variable        <=  omegaMax/omega.const 
    omega.variable        >=  omegaMin/omega.const 
    accel.variable     <=  accelMax/accel.const 
    accel.variable     >=  accelMin/accel.const 
    ];

Control_interval = 1:Iter_per_control_interval:numel(omega.variable);
for i = Control_interval
    constraints = [ constraints
        diff(omega.variable(i:i+Iter_per_control_interval-1)) == 0
        diff(accel.variable(i:i+Iter_per_control_interval-1)) == 0
        ];
end

%% Objective
objective = sum(diff(omega.physical(1:Iter_per_control_interval:N)).^2)/(N-Iter_per_control_interval-1) + ... %reduce control twitchiness
            sum(diff(accel.physical(1:Iter_per_control_interval:N)).^2)/(N-Iter_per_control_interval-1) + ... %reduce control twitchiness
            10*sum((th.physical(1:Iter_per_control_interval:N) - th_bar.physical(1:Iter_per_control_interval:N)).^2)/(N-Iter_per_control_interval) + ... %deviation from estimated ideal trajectory
            10*sum((v.physical(1:Iter_per_control_interval:N) - v_bar.physical(1:Iter_per_control_interval:N)).^2)/(N-Iter_per_control_interval); %deviation from estimated ideal trajectory

%% Collect sdpvars and return
s = whos;
for i = 1:numel(s)
    if strcmp(s(i).class,'mdlvar')
        variables.(s(i).name) = eval(s(i).name);
    end
end

