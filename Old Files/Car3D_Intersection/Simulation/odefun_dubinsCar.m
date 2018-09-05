function dydt = odefun_dubinsCar(t,y,u)
V = 5;
dydt = zeros(3,1);
dydt(1) = V*cos(y(3));
dydt(2) = V*sin(y(3));
dydt(3) = u(1);