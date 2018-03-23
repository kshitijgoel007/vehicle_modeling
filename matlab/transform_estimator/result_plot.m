function result_plot(n, tA, tE)
    
    %% Plot IMU to CG offset.
    figure;
    plot(0:1:n-1, tA(7,:), 'r--', 'LineWidth', 2);xlabel('Iteration', 'FontSize', 20);ylabel('r_{BC} (m)', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    plot(0:1:n-1, tA(8,:), 'g--', 'LineWidth', 2);
    plot(0:1:n-1, tA(9,:), 'b--', 'LineWidth', 2);
    
    p1 = plot(0:1:n-1, tE(7,:), 'r-', 'LineWidth', 2);
    p2 = plot(0:1:n-1, tE(8,:), 'g-', 'LineWidth', 2);
    p3 = plot(0:1:n-1, tE(9,:), 'b-', 'LineWidth', 2);
    
    legend([p1 p2 p3], {'x', 'y', 'z'});
    
    %% Plot learnt inertia.
    figure;
    plot(0:1:n-1, tA(4,:), 'r--', 'LineWidth', 2);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
    plot(0:1:n-1, tA(5,:), 'g--', 'LineWidth', 2);
    plot(0:1:n-1, tA(6,:), 'b--', 'LineWidth', 2);
    
    p4 = plot(0:1:n-1, tE(4,:), 'r-', 'LineWidth', 2);
    p5 = plot(0:1:n-1, tE(5,:), 'g-', 'LineWidth', 2);
    p6 = plot(0:1:n-1, tE(6,:), 'b-', 'LineWidth', 2);
    
    set(gca,'fontsize',9,'FontWeight','bold');xlabel('Iteration', 'FontSize', 14);ylabel('Inertia (kg m^2)', 'FontSize', 14);
    legend([p4 p5 p6], {'J_{xx}', 'J_{yy}', 'J_{zz}'});
end