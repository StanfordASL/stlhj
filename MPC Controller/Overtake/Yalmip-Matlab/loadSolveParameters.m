function BVs = loadSolveParameters(g)
%LOADBOUNDARYVALUES
% optimization parameters
BVs.T           = 1.;                      % [sec] Total time to hit target
BVs.dt           = 0.1;                 % [sec] Time step
BVs.nSS          = 1;                     % # of steps in SS constraint
BVs.N            = BVs.T/BVs.dt;         % # of time steps

% Initial conditions
BVs.x_0         = 0.2;                      % [m]     Initial x position
BVs.y_0         = -0.4;                     % [m]     Initial y position
BVs.th_0        = pi/2;                     % [rad]   Initial Heading position
BVs.v_0         = 0.15;                     % [m/s]   Initial Speed
BVs.y2_0        = -0.4;                     % [m/s]   Initial Dist. Car position
BVs.t_0         = 3;                        % [rad/s] Initial time

% States range
BVs.xMin   = g.min(1);
BVs.xMax   = g.max(1);
BVs.yMin   = g.min(2);
BVs.yMax   = g.max(2);
BVs.thMin  = g.min(3);
BVs.thMax  = g.max(3);
BVs.vMin   = g.min(4);
BVs.vMax   = g.max(4);

% Control range
BVs.omegaMin    = -0.4;
BVs.omegaMax    = 0.4;
BVs.accelMin    = -0.2;
BVs.accelMax    = 0.2;

end

