function newW = nextEstimate(QuadrotorModel, xNM, xActual, desRPM, tspan)
W = vertcat(xNM(1:10, 1), xNM(11:17, 1));
[t_soln, W_soln] = ode45(@(t,W) MAVDynamics(t, W(:,end), QuadrotorModel, desRPM'), tspan, W(:,end));
x = W_soln(end, :)';
newW = x(1:17, 1);
end