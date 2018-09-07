function u = PID_Controller(th,V,J_th,J_V,dt,v_mat,th_mat)
    [opt_th_I,~] = max(J_th);
    [opt_V_I,~] = max(J_V);
    
    opt_th = th_mat(opt_th_I);
    opt_V = v_mat(opt_V_I);
    
    error_th = opt_th-th;
    error_V = opt_V-V;
    
    prop = zeros(2,1);
    prop(1) = Kp_th*error_th;
    prop(2) = Kp_V*error_V;
    
    integral(1) = integral(1) + error_th*dt;
    integral(2) = integral(2) + error_V*dt;
    
    u(1) = Kp_th*error_th + Ki_th*integral(1);
    u(2) = Kp_V*error_V + Ki_V*integral(2);
    
end