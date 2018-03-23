function newW_IMU = nextEstimate_IMU(QuadrotorModel, xNM, xActual, a, omega, tspan)
% W = vertcat(xNM(1:10, 1), xActual(11:13, 1));
% [t_soln, W_soln] = ode45(@(t,W) IMUDynamics(t, W(:,end), a, omega, QuadrotorModel), tspan, W);
% x = W_soln(end, :)';
% newW_IMU = x(1:10, 1);

 % Discrete time system propagation using Euler-Forward Discretization.
 dt = 0.001;
 gravityVec = [0;0; 9.80665];
    newW_IMU(1:3, 1) = xNM(1:3, 1) + dt.*vectorTransform(xNM(4:6, 1), xNM(7:10, 1));
    newW_IMU(4:6, 1) = xNM(4:6, 1) + dt.*(a - rotationalTransform(xNM(7:10, 1))'*gravityVec - skewmat(omega)*xNM(4:6, 1));
    newW_IMU(7:10, 1) = quaternionComposition(xNM(7:10, 1), quaternionExponentialMap(dt.*omega));
end

function rT = vectorTransform(r, q)
    rT = (2*q(4,1)^2 - 1)*r + 2*q(4,1)*skewmat(q(1:3,1))*r + 2*q(1:3,1)*(q(1:3,1)'*r);
end

function R = rotationalTransform(q)
    R = (2*q(4,1)^2 - 1)*eye(3,3) + 2*q(4,1)*skewmat(q(1:3,1)) + 2*q(1:3,1)*q(1:3,1)';
end

function quatCom = quaternionComposition(q, p)
    q0 = q(4,1); p0 = p(4,1);
    qHat = q(1:3,1); pHat = p(1:3,1); 
    quatCom(4,1) = q0*p0 - qHat'*pHat; 
    quatCom(1:3,1) = q0*pHat + p0*qHat + cross(qHat, pHat);
end

function quatExp = quaternionExponentialMap(phi)
    if norm(phi) >= 0 && norm(phi) <= 1e-4
        quatExp(4, 1) = 1;
        quatExp(1:3, 1) = phi/2;
    else
        quatExp(4, 1) = cos(norm(phi)/2);
        quatExp(1:3, 1) = sin(norm(phi)/2)*phi/norm(phi);
    end  
end