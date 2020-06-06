function plot_step(output, t)
    Y1 = output(:, 1, 1) + output(:, 1, 2);
    Y2 = output(:, 2, 1) + output(:, 2, 2);
    
    figure;
    subplot(2, 1, 1);
    plot(t, Y1);
    title('Output Y1');
    xlabel('time [s]');
    ylabel('Amplitude');
    
    subplot(2, 1, 2);
    plot(t, Y2);
    title('Output Y2');
    xlabel('time [s]');
    ylabel('Amplitude');

end