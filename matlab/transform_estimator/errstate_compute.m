function errstate = errstate_compute(W1, W2, n)
    A = MultiplyQuat(W1(7:10,1), InverseQuat(W2(7:10,1)),1);
    errstate(7:9,1) = A(1:3, 1);
    errstate(1:3,1) = W1(1:3,1) - W2(1:3, 1);
    errstate(4:6,1) = W1(4:6,1) - W2(4:6, 1);
    if n == 1
     errstate(10:12,1) = W1(11:13,1) - W2(11:13, 1);
     errstate(13:16,1) = W1(14:17,1) - W2(14:17, 1);
    end
end