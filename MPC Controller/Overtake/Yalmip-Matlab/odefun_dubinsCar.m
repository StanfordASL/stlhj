function dydt = odefun_dubinsCar(t,y,u)
% V = 5;
% dydt = zeros(3,1);
% dydt(1) = V*cos(y(3));
% dydt(2) = V*sin(y(3));
% dydt(3) = u;

dydt = zeros(6,1);
dydt(1) = y(4)*cos(y(3));
dydt(2) = y(4)*sin(y(3))-0.01;
dydt(3) = u(1);
dydt(4) = u(2);
dydt(5) = u(3);
dydt(6) = 1;