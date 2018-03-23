% MOTOR MODEL
% Calculate coefficient of thrust for Shield vehicle

esc = [1200 1450 1600 1750 1850];

% Linear model
% (RPM) = m*(ESC) + b
m = 20.844;
b = -22688.774;
rpm = (m*esc + b)';

% Quadratic motor model
% (Thrust) = p1*(ESC)^2 + p2*(ESC) + p3
p1 = 1.3203889e-5;
p2 = -0.02958334;
p3 = 16.62471600;
T = (p1*((esc).^2) + p2*esc + p3)';

% Create system of equations
% (Thrust) = cT2*(RPM)^2 + cT1*(RPM) + cT0
A = zeros(length(rpm),3);
A(:,1) = rpm.^2;
A(:,2) = rpm;
A(:,3) = ones(length(rpm),1);

quadratic = A\T;