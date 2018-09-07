function [OptStates, MaxValueFunction] = loadOptimalStates(states,V1,V2,p,thd,vd,v2,integration,integration_iters)

OptStates.x_bar = [];
OptStates.y_bar = [];
OptStates.th_bar = [];
OptStates.v_bar = [];

t = linspace(states(6),states(6)+p.T,p.N+1);
dt = p.T/p.N;
%states_New = zeros(6,1);

for i = 1:numel(t)
    
    single_OptStates = loadSingleOptimalStates(states,V1,thd,vd);
    if  i == 1
        MaxValueFunction = 1;
    end

    omega = (single_OptStates(1) - states(3))/dt;
    if omega < p.omegaMin
        omega = p.omegaMin;
    elseif omega > p.omegaMax
        omega = p.omegaMax;
    end
    
    accel = (single_OptStates(2) - states(4))/dt;
    if accel < p.accelMin
        accel = p.accelMin;
    elseif accel > p.accelMax
        accel = p.accelMax;
    end
    
    dt2 = dt/integration_iters;
    if integration == 0 %Euler
        for i = 1:integration_iters
            states(1) = states(1) + states(4)*cos(states(3))*dt2;
            states(2) = states(2) + (states(4)*sin(states(3))-0.01)*dt2;
            states(3) = states(3) + omega*dt2;
            states(4) = states(4) + accel*dt2;
            states(5) = states(5) + v2*dt2;
            states(6) = states(6) + dt2;
            OptStates.th_bar = [OptStates.th_bar, single_OptStates(1)];
            OptStates.v_bar = [OptStates.v_bar, single_OptStates(2)];
        end
    elseif integration == 1 %Heun
        old_th = states(3);
        old_v = states(4);
        states(3) = states(3) + omega*dt;
        states(4) = states(4) + accel*dt;
        states(1) = states(1) + states(4)*cos(states(3))*dt/2 + old_v*cos(old_th)*dt/2;
        states(2) = states(2) + states(4)*sin(states(3))*dt/2 + old_v*sin(old_th)*dt/2;
        states(5) = states(5) + v2*dt;
        states(6) = states(6) + dt; 
        OptStates.th_bar = [OptStates.th_bar, single_OptStates(1)];
        OptStates.v_bar = [OptStates.v_bar, single_OptStates(2)];
    end
    
   OptStates.x_bar = [OptStates.x_bar, states(1)];
   OptStates.y_bar = [OptStates.y_bar, states(2)];
end
OptStates.th_bar = OptStates.th_bar(1:(numel(t)-1)*integration_iters+1);
OptStates.v_bar = OptStates.v_bar(1:(numel(t)-1)*integration_iters+1);

end

