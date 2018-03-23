function F = IMUDynamics(t, augW, a, omega, QuadModel)
    % Quaternion derivative
    q.x = augW(7);
    q.y = augW(8);
    q.z = augW(9);
    q.w = augW(10);
    R = QuatToR(q);
    F(1:3,1) = R*augW(4:6,1);
    F(4:6,1) = a - R'*[0;0;QuadModel.gravity] - skewmat(omega)*augW(4:6, 1);%[0;0;QuadModel.gravity] + R'*a;% - skewmat(omega)*augW(4:6,1);
    capitalOmega = [-skewmat(omega), omega;
                        -omega',        0];
    F(7:10,1) = 0.5.*(capitalOmega*augW(7:10));
    F(11:13,1) = zeros(3,1);
%     F(14:16,1) = randn(3, 1)*sigmaw;
%     F(17:19,1) = randn(3, 1)*sigmaw;
%     F(20:23,1) = zeros(4,1);
end