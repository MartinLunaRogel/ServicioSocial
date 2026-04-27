% Inicializar_Graficos_RRT.m
% =========================================================================
% === FUNCION PARA CONFIGURAR LA VENTANA Y DIBUJAR EL MAPA ===
% =========================================================================

function Inicializar_Graficos_RRT(x_limites, y_limites, inicio, objetivo, obstaculos)
    % Abre una nueva ventana de gráfico.
    figure;
    % Permite dibujar múltiples elementos en la misma ventana.
    hold on;
    % Muestra una cuadrícula para referencia.
    grid on;
    % Fija los límites de los ejes según los parámetros.
    xlim(x_limites);
    ylim(y_limites);

    % Dibuja el punto de inicio (verde) y llegada (rojo).
    plot(inicio(1), inicio(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(objetivo(1), objetivo(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

    % Añade títulos y etiquetas.
    title('Algoritmo RRT Bidireccional con Bias de Densidad y RRT*');
    xlabel('X'); ylabel('Y');

    % Itera sobre el cell array 'obstaculos' para dibujar cada figura (patch).
    for i = 1:size(obstaculos, 1)
        vertices = obstaculos{i, 1};
        patch(vertices(:,1), vertices(:,2), 'k', 'FaceAlpha', 0.3, 'EdgeColor', 'k');
    end
end
