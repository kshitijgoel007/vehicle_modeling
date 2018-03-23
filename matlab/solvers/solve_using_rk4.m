function [W_final, lin_ang_accel_save] = solve_using_rk4(t, W, QuadModel, desRPM, h)
    K = zeros(length(W), 4);
    K(:,1) = QuadrotorDynamics(t, W, QuadModel, desRPM);
    K(:,2) = QuadrotorDynamics(t + 0.5*h, W + 0.5*h*K(:,1), QuadModel, desRPM);
    K(:,3) = QuadrotorDynamics(t + 0.5*h, W + 0.5*h*K(:,2), QuadModel, desRPM);
    K(:,4) = QuadrotorDynamics(t + h, W + h*K(:,3), QuadModel, desRPM);
    
    q.x = W(7,1);
    q.y = W(8,1);
    q.z = W(9,1);
    q.w = W(10,1);
    
    R = QuatToR(q);
    lin_ang_accel_save(1:3,1) = K(4:6,1) + R'*[0; 0; 9.80665]  + cross(W(11:13,1), W(4:6,1));% 
    lin_ang_accel_save(4:6,1) = K(11:13,1);
    
    W_final = W + (1/6)*(K(:,1) + 2*K(:,2) + 2*K(:,3) + K(:,4))*h;
end
