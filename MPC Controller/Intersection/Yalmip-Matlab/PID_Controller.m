function [u,integral] = PID_Controller(th,V,J_th,J_V,dt,th_mat,V_mat,integral)

w_mat = [-0.4,0.4]; %range of allowable turning rate
a_mat = [-0.2,0.2]; %range of allowable acceleration

    Ki = 1;
    Kp = 2.5*Ki;
    
    Kp_th = Kp;
    Kp_V = Kp;
    
    Ki_th = Ki;
    Ki_V = Ki;
    
    [~,opt_th_I] = max(J_th);
    [~,opt_V_I] = max(J_V);
    
    opt_th = th_mat(opt_th_I);
    opt_V = V_mat(opt_V_I);
    
    error_th = opt_th-th;
    error_V = opt_V-V;
    
    integral_old = integral;
    integral(1) = integral(1) + error_th*dt;
    integral(2) = integral(2) + error_V*dt;
    
    u(1) = Kp_th*error_th + Ki_th*integral(1);
    u(2) = Kp_V*error_V + Ki_V*integral(2);
    
    if u(1)>w_mat(2)
        u(1) = w_mat(2);
        integral(1) = integral_old(1);
    elseif u(1)<w_mat(1)
        u(1) = w_mat(1);
        integral(1) = integral_old(1);
    end
    
    if u(2)>a_mat(2)
        u(2) = a_mat(2);
        integral(2) = integral_old(2);
    elseif u(2)<a_mat(1)
        u(2) = a_mat(1);
        integral(2) = integral_old(2);
    end
end