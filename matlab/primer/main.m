clear; clc; close all;

setpaths;

!synclient HorizTwoFingerScroll=0

% generate IMU data
fin_vect_actual = generateIMUdata(10);
%fin_vect_actual = final_struct_rosbags
fm = fin_vect_actual.U_IMU_accel;
wm = fin_vect_actual.U_IMU_omega;

% Measurement vector
rm = fin_vect_actual.Z(1:3, :);
qm = fin_vect_actual.Z(4:7, :);

% time vector and timestep
t = fin_vect_actual.timeMat;
dt = t(2) - t(1);

% IMU and Vicon data noise
sigmaf = 0.01; muf = 0.0;
sigmaw = 0.01; muw = 0.0;
sigmar = 0.05; mur = 0.0;
sigmaq = 0.05; muq = 0.0;
sigmabw = 0.01; mubw = 0.0;
sigmaba = 0.01; muba = 0.0;
sigmav = 0.001; muv = 0.0;

% State vector initialization
r = zeros(3, length(t)); % Position
r(1:3, 1) = [1; 0; 1];
v = zeros(3, length(t)); % Velocity
q = zeros(4, length(t)); % Quaternion
q(4,1) = 1;
bf = zeros(3, length(t)); % Acceleration bias
bw = zeros(3, length(t)); % Angular velocity bias

% Covariance and mean initialization
mu = zeros(16, length(t));
mu(:, 1) = vertcat(r(:, 1), v(:, 1), q(:, 1), bf(:, 1), bw(:, 1));
cov = zeros(15,15, length(t));
cov(:, :, 1) = 0.01.*eye(15,15);

% Other initializations
gravityVec = [0; 0; 9.80665];

% Simulation loop
for i=2:1:length(t)-1
    nf = rand(3, 1)*sigmaf + muf; % Zero mean gaussian noise in IMU acceleration
    nw = rand(3, 1)*sigmaw + muw; % Zero mean gaussian noise in IMU angular velocity
    nvr = rand(3, 1)*sigmar + mur; % Zero mean gaussian noise in Vicon position measurement
    nvq = rand(3, 1)*sigmaq + muq; % Zero mean gaussian noise in Vicon quaternion measurement
    nvba = rand(3, 1)*sigmaba + muba; % Zero mean gaussian noise process for bias in accel.
    nvbw = rand(3, 1)*sigmabw + mubw; % Zero mean gaussian noise process for bias in angular velocity;
    nvv = rand(3, 1)*sigmav + muv; % % Zero mean gaussian noise process for linear velocity;
    
    % Zero mean gaussian noise in IMU angular velocity
    f(:, i) = fm(:, i) - bf(:, i) - nf;
    w(:, i) = wm(:, i) - bw(:, i) - nw;

    % Discrete time system propagation using Euler-Forward Discretization.
    r(:, i) = r(:, i-1) + dt.*vectorTransform((v(:, i-1) + nvv), q(:, i-1));
    v(:, i) = v(:, i-1) + dt.*(f(:, i) - rotationalTransform(q(:, i-1))'*gravityVec - skewmat(w(:,i))*v(:, i-1));
    q(:, i) = quaternionComposition(q(:, i-1), quaternionExponentialMap(dt.*w(:, i-1)));
    bf(:, i) = bf(:, i-1) + dt.*(nvba);
    bw(:, i) = bw(:, i-1) + dt.*(nvbw);
    
    % Add noise to Vicon measurements.
    rm(:, i) = rm(:, i) + nvr;
    qm(:, i) = quaternionComposition(quaternionExponentialMap(nvq), qm(:, i));
    
    % Jacobian calculation
    sys.F = [ eye(3,3), dt.*rotationalTransform(q(:, i)), -dt.*(skewmat(vectorTransform(v(:, i), q(:, i)))), zeros(3, 3), zeros(3,3); 
          zeros(3,3), eye(3,3) - dt.*(skewmat(w(:, i))), dt.*rotationalTransform(q(:, i)), -dt.*eye(3,3), -dt.*skewmat(v(:, i)) ;
          zeros(3,3), zeros(3,3), eye(3,3), zeros(3,3), -dt.*rotationalTransform(q(:, i))*phiJacobian(dt.*w(:,i));
          zeros(3,3), zeros(3,3), zeros(3,3), eye(3,3), zeros(3,3);
          zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), eye(3,3)]; % Jacobian w.r.t state for process model.
      
    sys.H = [eye(3,3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3); 
         zeros(3,3), zeros(3,3), eye(3,3), zeros(3,3), zeros(3,3)]; % Jacobian w.r.t state for measurement model.
     
    sys.Q = [(sigmar^2).*eye(3,3), zeros(3,3);
             zeros(3,3), (sigmaq^2).*eye(3,3)]; % Jacobian w.r.t noise vector for measurement model.
     
%     sys.R = [dt.*rotationalTransform(q(:, i)), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3);
%              zeros(3,3), -dt.*eye(3,3), -dt.*skewmat(v(:, i)), zeros(3,3), zeros(3,3);
%              zeros(3,3), zeros(3,3), -dt.*rotationalTransform(q(:, i))*phiJacobian(dt.*w(:,i)), zeros(3,3), zeros(3,3);
%              zeros(3,3), zeros(3,3), zeros(3,3), dt.*eye(3,3), zeros(3,3);
%              zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), dt.*eye(3,3)]; 
    sys.R = [(sigmav^2).*eye(3,3), zeros(3, 3), zeros(3, 3), zeros(3, 3), zeros(3, 3);
             zeros(3, 3), (sigmaf^2).*eye(3, 3),zeros(3, 3),zeros(3, 3),zeros(3, 3);
             zeros(3, 3),zeros(3, 3), (sigmaw^2).*eye(3, 3),zeros(3, 3),zeros(3, 3);
             zeros(3, 3),zeros(3, 3),zeros(3, 3), (sigmaba^2).*eye(3, 3),zeros(3, 3);
             zeros(3, 3),zeros(3, 3),zeros(3, 3),zeros(3, 3), (sigmabw^2).*eye(3, 3)]; % Jacobian w.r.t noise vector for process model.
    
    % State after process update
    mu(:, i) = vertcat(r(:, i), v(:, i), q(:, i), bf(:, i), bw(:, i));
    
    % Measurement vector for innovation computation
    z = vertcat(rm(:, i), qm(:, i));
    
    % Call EKF correction update
    [mu(:, i), cov(:,:,i)] = EKFCorrection(mu(:, i), cov(:,:,i-1), z, sys);
    
    % Reassign states after correction update.
    r(:, i) = mu(1:3, i);
    v(:, i) = mu(4:6, i);
    q(:, i) = mu(7:10, i);
    bf(:, i) = mu(11:13, i);
    bw(:, i) = mu(14:16, i);
end

states_ = vertcat(t(1:length(w))', mu(:, 1:length(w)));   
plot_state(states_', cov, fin_vect_actual.X(:, 1:length(w))');

%% Rotation of a vector corresponding to a quaternion.
function rT = vectorTransform(r, q)
    rT = (2*q(4,1)^2 - 1)*r + 2*q(4,1)*skewmat(q(1:3,1))*r + 2*q(1:3,1)*(q(1:3,1)'*r);
end

%% Rotational transform corresponding to a quaternion.
function R = rotationalTransform(q)
    R = (2*q(4,1)^2 - 1)*eye(3,3) + 2*q(4,1)*skewmat(q(1:3,1)) + 2*q(1:3,1)*q(1:3,1)';
end

%% Quaternion composition q o p.
function quatCom = quaternionComposition(q, p)
    q0 = q(4,1); p0 = p(4,1);
    qHat = q(1:3,1); pHat = p(1:3,1); 
    quatCom(4,1) = q0*p0 - qHat'*pHat; 
    quatCom(1:3,1) = q0*pHat + p0*qHat + cross(qHat, pHat);
end

%% Quaternion Exponential Map. phi (R3) -> quatExp (SO(3), 4x1)
function quatExp = quaternionExponentialMap(phi)
    if norm(phi) >= 0 && norm(phi) <= 1e-4
        quatExp(4, 1) = 1;
        quatExp(1:3, 1) = phi/2;
    else
        quatExp(4, 1) = cos(norm(phi)/2);
        quatExp(1:3, 1) = sin(norm(phi)/2)*phi/norm(phi);
    end  
end

%% Quaternion Logarithmic Map. quatExp (SO(3), 4x1) -> phi (R3)
function quatLog = quaternionLogarithmicMap(quat)
    if norm(quat(1:3, 1)) >= 0 && norm(quat(1:3, 1)) <= 1e-4
        quatLog(1:3, 1) = sign(quat(4, 1)).*quat(1:3, 1);
    else
        quatLog(1:3, 1) = 2*atan2(norm(quat(1:3, 1)), quat(4, 1))*quat(1:3, 1)/norm(quat(1:3, 1));
    end  
end

%% Inverse of Quaternion.
function quatInverse = quaternionInverse(quat)
    quatInverse(4,1) = quat(4, 1);
    quatInverse(1:3, 1) = (-1).*quat(1:3, 1);
    quatInverse = quatInverse/norm(quatInverse);
end

%% Box Operator for substraction.
function vec = boxSub(vec_, a_)
    vec(1:3, 1) = vec_(1:3, 1) - a_(1:3, 1);
    vec(4:6, 1) = quaternionLogarithmicMap(quaternionComposition(vec_(4:7, 1), quaternionInverse(a_(4:7, 1))));  
end

%% Box Operator for addition.
function vec = boxAdd(vec_, a_)
    vec(1:6, 1) = vec_(1:6, 1) + a_(1:6, 1);
    vec(11:16, 1) = vec_(11:16, 1) + a_(10:15, 1);
    vec(7:10, 1) = quaternionComposition(quaternionExponentialMap(a_(7:9, 1)), vec_(7:10, 1)); 
end

%% EKF Update
function [mu, cov] = EKFCorrection(mu_, cov_, z_, sys)
    covTemp = sys.F*cov_*sys.F' + sys.R;
    K = covTemp*sys.H'*inv(sys.H*covTemp*sys.H' + sys.Q);
    innov = boxSub(z_, [mu_(1:3, 1); mu_(7:10, 1)]);
    mu = boxAdd(mu_, K*innov);
    cov = (eye(size(covTemp)) - K*sys.H)*covTemp;
end

%% IMU data generation
function fin_vect_actual = generateIMUdata(t_final)
    prompt = 'Timestep: ';
    h = 0.001; %input(prompt);
    prompt = 'Solver- 0 for RK4, 1 for ODE45: ';
    solver = 0; %input(prompt);
    prompt = 'Plot results from dynamics simulation? Yes(1)/No(0) ';
    plotResults = 0;%input(prompt);

    % Model Quadrotor that has to be iterated.
    hulk1c = QuadrotorModel;

    if h == 0
        disp('Invalid timestep. Aborting..');
    end

    str = {'experimental_try', ...
           'Circle', ...
           'Leminiscate', ...
           'Eight Curve', ...
           'Ellipse', ...
           'Cornoid', ...
           'Astroid', ...
           'Vertical Circle', ...
           'Bicorn', ...
           'Butterfly Curve', ...
           'TakeOff', ...
           'Spiral Circle', ...
           'Circle_MultiDimensional', ...
           'Back&Forth'};

[s,v] = listdlg('PromptString','Select a trajectory:',...
                 'SelectionMode','single','ListSize',[300 300],...
                 'ListString',str);
if v
    trajectory = str{s};
 else
     disp('No behavior selected. Aborting..')
     return
end

% Initialization
% Parameter struct
thetaActual.cT = 1.7081e-5; % Coefficient of Thrust
thetaActual.cTorque = 0.011651904893177;
thetaActual.cd = 0;
thetaActual.I = [0.0058508, 0, 0;
                 0, 0.0081457, 0;
                 0, 0, 0.0126617]; % Moment of Inertia
thetaActual.rOffset = [0.0008; 0.0019; 0.0];
thetaActual.imu_offset = [0.0; 0.0; 0.0]; %[-0.0726; -0.0012; -0.018];

mes2 = ['Running dynamics simulation for ', num2str(t_final), ' seconds using actual parameters ...'];
disp(mes2);
fin_vect_actual = quad_dyn_main(thetaActual, solver, t_final, h, trajectory, plotResults);
end

%% Tangent space Jacobian.
function gamma = phiJacobian(phi)
    if norm(phi) >= 0 && norm(phi) <= 1e-4
        gamma = eye(3,3) + 0.5.*(skewmat(phi));
    else
        gamma = eye(3,3) + (1 - cos(norm(phi)))*skewmat(phi)/((norm(phi))* norm(phi)) ...
            + (norm(phi) - sin(norm(phi)))*skewmat(phi)*skewmat(phi)/(norm(phi)* norm(phi) * norm(phi));
    end  
end

%% Plot states.
function plot_state(states, cov, xa)
    % Euler angles and Position error bounds
    presentQuat = states(:, 8:11);
    for i=1:length(presentQuat)
        eulerStates(i, :) = QuatToZYX(presentQuat(i,:));
        sigmaStates(i, :) = 3.*sqrt(diag(cov(:,:, i)));
        eulerActual(i, :) = QuatToZYX(xa(i, 7:10));
    end

    r2d = 180/pi;
    
    % Plot desired and actual euler angles
    figure;
    subplot(3,1,1);shadedErrorBar(states(:,1),eulerStates(:,1)*r2d,  sigmaStates(:, 7)*r2d, 'lineprops', 'r');xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    grid on; 
    hold on;
    subplot(3,1,1);plot(states(:,1), eulerActual(:,1)*r2d, 'LineWidth', 1.5);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('estimator 3 \sigma', 'estimator mean', 'actual');
    subplot(3,1,2);shadedErrorBar(states(:,1),eulerStates(:,2)*r2d,  sigmaStates(:, 8)*r2d, 'lineprops', 'g');xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
    grid on; 
    hold on;
    subplot(3,1,2);plot(states(:,1), eulerActual(:,2)*r2d, 'LineWidth', 1.5);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('estimator 3 \sigma', 'estimator mean', 'actual');
    
    subplot(3,1,3);shadedErrorBar(states(:,1),eulerStates(:,3)*r2d,  sigmaStates(:, 9)*r2d, 'lineprops', 'b');xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
    grid on; 
    hold on;
    subplot(3,1,3);plot(states(:,1), eulerActual(:,3)*r2d, 'LineWidth', 1.5);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('estimator 3 \sigma', 'estimator mean', 'actual');
     
%    Plot desired and actual position
    figure;
    subplot(3,1,1);shadedErrorBar(states(:,1), states(:,2), sigmaStates(:, 1), 'lineprops', 'r');xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    grid on; 
    hold on;
    subplot(3,1,1);plot(states(:,1), xa(:,1), 'LineWidth', 1.5);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('estimator 3 \sigma', 'estimator mean', 'actual');

    subplot(3,1,2);shadedErrorBar(states(:,1), states(:,3), sigmaStates(:, 2), 'lineprops', 'g');xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
    grid on; 
    hold on;
    subplot(3,1,2);plot(states(:,1), xa(:,2), 'LineWidth', 1.5);
    legend('estimator 3 \sigma', 'estimator mean', 'actual');

    set(gca,'fontsize',12,'FontWeight','bold');
    subplot(3,1,3);shadedErrorBar(states(:,1), states(:,4), sigmaStates(:, 3), 'lineprops', 'b');xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
    legend('estimator 3 \sigma', 'estimator mean', 'actual');

    grid on;
    hold on;
    subplot(3,1,3);plot(states(:,1), xa(:,3), 'LineWidth', 1.5);
    set(gca,'fontsize',12,'FontWeight','bold');
    legend('estimator 3 \sigma', 'estimator mean', 'actual');

    % Plot desired and actual linear velocities
    figure;
    subplot(3,1,1);plot(states(:,1),states(:,5), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('v_{x}', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');

    subplot(3,1,2);plot(states(:,1),states(:,6), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('v_{y}', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    
    subplot(3,1,3);plot(states(:,1),states(:,7), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('v_{z}', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');

    % Plot acceleration bias term.
    figure;
    subplot(3,1,1);plot(states(:,1), states(:, 12), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('a_{x}', 'FontSize', 20);
    grid on;
    set(gca, 'fontsize', 12, 'FontWeight', 'bold');
    title('Acceleration bias v/s time');

    subplot(3,1,2);plot(states(:,1), states(:, 13), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('a_{y}', 'FontSize', 20);
    grid on;
    set(gca, 'fontsize', 12, 'FontWeight', 'bold');

    subplot(3,1,3);plot(states(:,1), states(:, 14), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('a_{z}', 'FontSize', 20);
    grid on;
    set(gca, 'fontsize', 12, 'FontWeight', 'bold');
    
    % Plot angular velocity bias term.
    figure;
    subplot(3,1,1);plot(states(:,1), states(:, 15), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('w_{x}', 'FontSize', 20);
    grid on;
    set(gca, 'fontsize', 12, 'FontWeight', 'bold');
    title('Angular velocity bias v/s time');

    subplot(3,1,2);plot(states(:,1), states(:, 16), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('w_{y}', 'FontSize', 20);
    grid on;
    set(gca, 'fontsize', 12, 'FontWeight', 'bold');

    subplot(3,1,3);plot(states(:,1), states(:, 17), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('w_{z}', 'FontSize', 20);
    grid on;
    set(gca, 'fontsize', 12, 'FontWeight', 'bold');
end
