function F = get_nominal_data(QuadrotorModel, desRPM, init, t)
    W(:,1) = init;
    for i = 1:1:length(t)-1
        tspan = [t(i) t(i+1)];
        [t_soln, W_soln] = ode45(@(t,W) QuadrotorDynamics(t, W(:,end), QuadrotorModel, desRPM(i, :)), tspan, W(:,end));
        W(:,i+1) = W_soln(end,:)';
    end
    F.X = W;
end