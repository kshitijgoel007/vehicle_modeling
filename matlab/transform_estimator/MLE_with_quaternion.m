clear;
clc;
close all;

!synclient HorizTwoFingerScroll=0

addpath('../sensor_models/');
addpath('../controllers');
addpath('../solvers');
addpath('../trajectory_generator');
addpath('../classes');
addpath('../utilities');
addpath('../');
addpath('../../matlab_utils/src/');

%% User commands : Final time, time step, solver, and trajectory selection
prompt = 'Final time: ';
t_final = 5; %input(prompt);
prompt = 'Timestep: ';
h = 0.001; %input(prompt);
prompt = 'Solver- 0 for RK4, 1 for ODE45: ';
solver = 0; %input(prompt);
prompt = 'Plot results from dynamics simulation? Yes(1)/No(0) ';
plotResults = 1;%input(prompt);

%% Model Quadrotor that has to be iterated.
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
%       trajectory = 'Circle';
 else
     disp('No behavior selected. Aborting..')
     return
end

%% Initialization
% Parameter struct
thetaActual.cT = 1.7081e-5; % Coefficient of Thrust
thetaActual.cTorque = 0.011651904893177;
thetaActual.cd = 0;
% thetaActual.I = [0.0058508, -6.5e-5, -0.00029204;
%                 -6.5e-5,   0.008 + 0.0081457, 3.528e-5;
%                  -0.00029204, 3.528e-5, 0.0126617]; % Moment of Inertia
thetaActual.I = [0.0058508, 0, 0;
                0, 0.008 + 0.0081457, 0;
                0, 0, 0.0126617]; % Moment of Inertia
thetaActual.rOffset = [0.0; 0.0; 0.0];
thetaActual.imuOffset = [0.0; 0.0; 0.0];

mes2 = ['Running dynamics simulation for ', num2str(t_final), ' seconds using actual parameters ...'];
disp(mes2);
fin_vect_actual = quad_dyn_main(thetaActual, solver, t_final, h, trajectory, plotResults);
close all;
U = fin_vect_actual.FTotalMTotal;

thetaNominal.cT = 1.7081e-5; % Coefficient of Thrust
thetaNominal.cTorque = 0.011651904893177;
thetaNominal.cd = 0;
% thetaNominal.I = [0.0058508, -6.5e-5, -0.00029204;
%                  -6.5e-5,   0.0081457, 3.528e-5;
%                  -0.00029204, 3.528e-5, 0.0126617]; % Moment of Inertia
thetaNominal.I = [0.0058508, 0, 0;
                0,  0.0081457, 0;
                0, 0, 0.0126617]; % Moment of Inertia
thetaNominal.rOffset = [0.0; 0.0; 0.0];

hulkNominal = QuadrotorModel;
hulkNominal = hulkNominal.setThetaVal(thetaNominal);

mes = ['Running dynamics simulation for ', num2str(t_final), ' seconds using nominal parameters ...'];
disp(mes);

t = fin_vect_actual.timeMat;

fin_vect = get_nominal_data(hulkNominal, fin_vect_actual.commandRPM', fin_vect_actual.X(:, 1), t);

% Vicon Measurement Vector
z(1:7,:) = fin_vect_actual.Z(1:7,:);     % Measurement vector.    
prompt = 'Number of iterations for MLE: ';
n = 7;%input(prompt);

mes3 = ['Running Maximum Likelihood Estimator for ', num2str(n), ' iterations to learn actual params from nominal. Please wait ...'];
disp(mes3);

%% MLE specific initialization
xNominal = fin_vect.X(1:17,:);
xActual = fin_vect_actual.X;

a_IMU = fin_vect_actual.U_IMU_accel;
omega_IMU = fin_vect_actual.U_IMU_omega;

dt = t(2) - t(1);

% Estimated parameter vector.
thetaEst = zeros(9,1);
thetaEst(1:3) = [thetaNominal.cT; thetaNominal.cTorque; thetaNominal.cd];
thetaEst(4:6) = [thetaNominal.I(1,1); thetaNominal.I(2,2); thetaNominal.I(3,3)];
thetaEst(7:9) = thetaNominal.rOffset;

thetaA = zeros(9, n);
thetaA(1:3, 1) = [thetaActual.cT; thetaActual.cTorque; thetaActual.cd];
thetaA(4:6, 1) = [thetaActual.I(1,1); thetaActual.I(2,2); thetaActual.I(3,3)];
thetaA(7:9, 1) = thetaActual.rOffset;

% Negative log likelihood function
L = zeros(n-1, 1);

% Noise Matrices
R = 0.01*eye(6,6);
Q = 0.01*eye(16,16);

I = 0.01*eye(9,9);

% Declaring size of jacobians and least square results.
FM = zeros(16,16,length(t));
FI = zeros(9,9,length(t));
%computedCorrections = zeros(22, length(t), n);
thetaFinal(:, 1) = thetaEst;

for ii = 1:1:n-1
    for i = 2:1:length(t)
        % vars for jacobian calculation
        vars = vertcat(xNominal(1:17, i-1), thetaEst, fin_vect_actual.commandRPM(:, i-1), hulkNominal.gravity, ...
            hulkNominal.mass, 1/hulkNominal.kMotor, omega_IMU(i-1, :)');

        % Calculate jacobians at the current nominal state vector
        [FM(:,:,i-1), JTheta, FI(:,:,i-1)] = fast_jac_substitution(vars);
        H = [eye(3,3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,4);
            zeros(3,3), zeros(3,3), eye(3,3), zeros(3,3), zeros(3,4)];

        distrans = expm(dt.*FM(:,:,i-1));
        distranstheta = ones(16,1) + dt.*JTheta;
        distransIMU = expm(dt.*FI(:,:,i-1));

        % Forward propagating MAV dynamics model
        
        theta.cT = thetaEst(1); % Coefficient of Thrust
        theta.cTorque = thetaEst(2);
        theta.cd = thetaEst(3);
        theta.I = diag(thetaEst(4:6));
        theta.rOffset = thetaEst(7:9);
        
        hulk1c = hulk1c.setThetaVal(theta);
        
        tspan = [t(i-1) t(i)];
        
        % Solve Least Squares
        if i == 2
        rx_temp = nextEstimate(hulk1c, xNominal(:, i-1), xActual(:, i-1), fin_vect_actual.commandRPM(:, i-1), tspan);
        %rx = errstate_compute(xActual(1:17, i), xNominal(:, i), 1); 
        rx = errstate_compute(xNominal(:, i), rx_temp, 1); 
        
        % IMU residual
        ri_temp = nextEstimate_IMU(hulk1c, xNominal(:, i-1), xActual(:, i-1), a_IMU(i-1,:)', omega_IMU(i-1,:)', tspan);
        ri = errstate_compute(xNominal(:,i), ri_temp, 0);
        
        % Residual calculation
        rz = min_errstate_compute(z(:,i), [xNominal(1:3, i); xNominal(7:10, i)]);
        
        r = [rz; rx; ri];
        A = [H, zeros(6, 1); distrans, distranstheta; distransIMU, zeros(9,8)];
        else
            
        A = vertcat(A,[H, zeros(6, 1); distrans, distranstheta; distransIMU, zeros(9,8)]);
        rx_temp = nextEstimate(hulk1c, xNominal(:, i-1), xActual(:, i-1), fin_vect_actual.commandRPM(:, i-1), tspan);
        %rx = errstate_compute(xActual(1:17, i), xNominal(:, i), 1); 
        rx = errstate_compute(xNominal(:, i), rx_temp, 1); 
        
        % IMU residual
        ri_temp = nextEstimate_IMU(hulk1c, xNominal(:, i-1), xActual(:, i-1), a_IMU(i-1,:)', omega_IMU(i-1,:)', tspan);
        ri = errstate_compute(xNominal(:,i), ri_temp, 0);
        
        % Residual calculation
        rz = min_errstate_compute(z(:,i), [xNominal(1:3, i); xNominal(7:10, i)]);
        
        r = vertcat(r, [rz; rx; ri]);
        end
        computedCorrections = pinv(A)*r;
        xNominal(:, i) = propagate_state(xNominal(:, i), computedCorrections(1:16), 1);
        thetaEst(5) = thetaEst(5) + computedCorrections(17, 1);
        thetaEst(1:3) = thetaEst(1:3);
        thetaEst(7:9) = thetaEst(7:9);% + computedCorrections(20:22);
       disp(t(i)); 
    end
    thetaA(:,ii+1) = thetaA(:,ii);
    thetaFinal(:, ii + 1) = thetaEst;
    %disp(ii);
end

result_plot(n, thetaA, thetaFinal);
