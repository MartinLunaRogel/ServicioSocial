function graficar_distribucion(valores_x, valores_y)

    % Crea la figura para la gráfica de distribución
    figure;

    % Grafica el histograma de las coordenadas X en azul
    hist(valores_x, 'FaceColor', 'blue', 'DisplayName', 'Coordenadas X');
    hold on; % Mantiene el gráfico para poder añadir el segundo histograma

    % Grafica el histograma de las coordenadas Y en rojo
    hist(valores_y, 'FaceColor', 'red', 'FaceAlpha', 0.5, 'DisplayName', 'Coordenadas Y');

    title('Distribución de Puntos Aleatorios con Muestreo del Objetivo');
    xlabel('Valor de la Coordenada');
    ylabel('Frecuencia');
    legend('show');
    grid on;
    hold off;
end
