function errstate = min_errstate_compute(W1, W2)
    A = MultiplyQuat(W1(4:7,1), InverseQuat(W2(4:7,1)),1);
    errstate(4:6,1) = 2.*A(1:3, 1);
    errstate(1:3,1) = W1(1:3,1) - W2(1:3, 1);
end