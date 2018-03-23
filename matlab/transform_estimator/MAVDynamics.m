%% Nonlinear 6DOF dynamics for a quadrotor
% Dynamics model like QuadrotorDynamics.m with CG offset from geometric
% center included

function [F] = MAVDynamics(t, W, QuadModel, desRPM)
%% Initialize F and params
F = zeros(17,1);

[U] = QuadModel.constrained_control_inputs(W(14:17));

%% Rotational Dynamics
tau = U(2:4);
angVel = W(11:13);
F(11:13) = QuadModel.AngularDynamics(tau, angVel, U);

%% Translational dynamics in world frame
F(4:6) = QuadModel.TranslationalDynamics(W, U, F(11:13));

%% Linear velocities in world frame
q.x = W(7);
q.y = W(8);
q.z = W(9);
q.w = W(10);

F(1:3) = QuatToR(q)*W(4:6);

%% Quaternion Rates
capitalOmega = [   0,    W(13), -W(12), W(11); ...
               -W(13),       0,  W(11), W(12); ...
                W(12),   -W(11),     0, W(13); ...
               -W(11),   -W(12),  -W(13),   0];
overallCapital_omega = capitalOmega/2;

F(7:10) = overallCapital_omega*W(7:10);

%% Motor dynamics
F(14:17) = -QuadModel.kMotor*(W(14:17) - desRPM');

end
