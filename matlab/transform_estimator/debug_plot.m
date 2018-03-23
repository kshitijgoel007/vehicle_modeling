function debug_plot(Y, states, z)
    %Position
    figure;
    grid on;
    subplot(3,1,1);plot(states(:,1),states(:,2), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,1);plot(Y(:,1),Y(:,2), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,1);plot(states(:,1),z(:,1), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    grid on;
    set(gca,'fontsize',12,'FontWeight','bold');
    hold off;
    legend('actual', 'desired', 'z');
    
    subplot(3,1,2);plot(states(:,1),states(:,3) , 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,2);plot(Y(:,1),Y(:,3) , 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Y', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,2);plot(states(:,1),z(:,2), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    grid on;
    set(gca,'fontsize',12,'FontWeight','bold');        
    hold off;
    legend('actual', 'desired', 'z');
    
    subplot(3,1,3);plot(states(:,1),states(:,4) , 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,3);plot(Y(:,1),Y(:,4) , 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('Z', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,3);plot(states(:,1),z(:,3), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('X', 'FontSize', 20);
    grid on;
    set(gca,'fontsize',12,'FontWeight','bold');
    
    hold off;
    legend('actual', 'desired', 'z');
    
    % Velocity
    figure;
    grid on;
    subplot(3,1,1);plot(states(:,1),states(:,5), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('V_X', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,1);plot(Y(:,1),Y(:,5), 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('V_X', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold off;
    legend('actual', 'desired');
    subplot(3,1,2);plot(states(:,1),states(:,6) , 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('V_Y', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,2);plot(Y(:,1),Y(:,6) , 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('V_Y', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold off;
    legend('actual', 'desired');
    subplot(3,1,3);plot(states(:,1),states(:,7) , 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('V_Z', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,3);plot(Y(:,1),Y(:,7) , 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('V_Z', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold off;
    legend('actual', 'desired');
    
    % Quaternion to ZYX for states
    for i = 1:length(states(:,1))
        qActual = states(i, 8:11)';
        ZYXActual(i, :) = QuatToZYX(qActual);
        
        qDesired = Y(i, 8:11)';
        ZYXDesired(i, :) = QuatToZYX(qDesired);
    end

    % Quaternion to ZYX for Z
    for i = 1:length(z(:,1))
        qActualforZ = z(i, 4:7)';
        ZinQuat(i, :) = QuatToZYX(qActualforZ);        
%         qDesired = Y(i, 8:11)';
%         ZYXDesired(i, :) = QuatToZYX(qDesired);
    end    
    
    r2d = 180/pi;
    % Plot desired and actual euler angles
    grid on;
    figure;
    subplot(3,1,1);plot(states(:,1), ZYXActual(:,1)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,1);plot(Y(:,1),ZYXDesired(:,1)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,1);plot(states(:,1), ZinQuat(:,1)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    grid on;
    set(gca,'fontsize',12,'FontWeight','bold');
    
    hold off;
    legend('actual', 'desired', 'z');
    subplot(3,1,2);plot(states(:,1),ZYXActual(:,2)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,2);plot(Y(:,1),ZYXDesired(:,2)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\theta', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,2);plot(states(:,1), ZinQuat(:,2)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    grid on;
    set(gca,'fontsize',12,'FontWeight','bold');
    
    hold off;
    legend('actual', 'desired', 'z');
    subplot(3,1,3);plot(states(:,1),ZYXActual(:,3)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,3);plot(Y(:,1), ZYXDesired(:,3)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\psi', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,3);plot(states(:,1), ZinQuat(:,3)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('\phi', 'FontSize', 20);
    grid on;
    set(gca,'fontsize',12,'FontWeight','bold');
    
    hold off;
    legend('actual', 'desired', 'z');
    
    % Plot desired and actual angular velocities
    figure;
    subplot(3,1,1);plot(states(:,1),states(:,12)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('p', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,1);plot(Y(:,1),Y(:,12)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('p', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold off;
    legend('actual', 'desired');
    subplot(3,1,2);plot(states(:,1),states(:,13)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('q', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,2);plot(Y(:,1),Y(:,13)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('q', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold off;
    legend('actual', 'desired');
    subplot(3,1,3);plot(states(:,1),states(:,14)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('r', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    subplot(3,1,3);plot(Y(:,1),Y(:,14)*r2d, 'LineWidth', 1.5);xlabel('time', 'FontSize', 20);ylabel('r', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold off;
    legend('actual', 'desired');
end