function W_ = propagate_state(W, E, n)
% Error quaternion normalization
%     if (0.5.*E(7:9,1)')*(0.5.*E(7:9,1)) > 1
%         delqIG = (1/sqrt(1 + (0.5.*E(7:9,1)')*(0.5.*E(7:9,1)))).*[0.5.*E(7:9,1); 1];
%     else
%         delqIG = [0.5.*E(7:9,1); sqrt(1 - (0.5.*E(7:9,1)')*(0.5.*E(7:9,1)))];
%     end
    if norm(E(7:9,1)) < 1e-3 && norm(E(7:9,1)) >= 0
    delqIG = [0.5.*E(7:9,1); 1];
    else
    delqIG = [sin(norm(E(7:9,1))/2)*E(7:9,1)/norm(E(7:9,1)) ; cos(norm(E(7:9,1))/2)];
    end
    
    W_(7:10, 1) = MultiplyQuat((W(7:10, 1)), delqIG, 1);
    W_(1:6, 1) = W(1:6, 1) + E(1:6, 1);
    if n == 1
     W_(11:13, 1) = W(11:13, 1) + E(10:12, 1);
     W_(14:17, 1) = W(14:17, 1) + E(13:16, 1);
    end
end
