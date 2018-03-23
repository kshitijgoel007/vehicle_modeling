function result_plot_z(n, tA, tE_Spiral, tE_Circle, tE_Leminiscate, tE_Bicorn)
    figure;
    plot(0:1:n-1, tA(9,:), 'r--', 'LineWidth', 2);xlabel('Iteration', 'FontSize', 20);ylabel('r_{BC} (m)', 'FontSize', 20);
    grid on; 
    set(gca,'fontsize',12,'FontWeight','bold');
    hold on;
%     plot(0:1:n-1, tA(8,:), 'g--', 'LineWidth', 2);
%     plot(0:1:n-1, tA(9,:), 'b--', 'LineWidth', 2);
    
    p1_Spiral = plot(0:1:n-1, tE_Spiral(9,:), 'r-', 'LineWidth', 2);
    
    hold on;
    p2_Circle = plot(0:1:n-1, tE_Circle(9,:), 'g-', 'LineWidth', 2);
    
    hold on;
    p3_Leminiscate = plot(0:1:n-1, tE_Leminiscate(9,:), 'b-', 'LineWidth', 2);
    
    hold on;
    p4_Bicorn = plot(0:1:n-1, tE_Bicorn(9,:), 'm-', 'LineWidth', 2);
%     p2 = plot(0:1:n-1, tE(8,:), 'g-', 'LineWidth', 2);
%     p3 = plot(0:1:n-1, tE(9,:), 'b-', 'LineWidth', 2);
    
    legend('Ground Truth','Spiral Circle Trajectory','Circle Trajectory', ...
            'Leminiscate Trajectory','Bicorn Trajectory','Location','northeast');
    title('Offset IMU to CoG r_{BC}^{B} in Z direction')
    set(gca,'fontsize',12,'FontWeight','bold');
end