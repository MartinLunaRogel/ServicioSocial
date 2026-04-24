% Dibujar_Ruta_Final.m
% =========================================================================
% === FUNCION PARA CONCATENAR Y DIBUJAR EL CAMINO COMPLETO ===
% =========================================================================

function path_completo = Dibujar_Ruta_Final(coords_inicio, coords_objetivo, estilo)
    % Inversa la rama del objetivo y quita el primer nodo para evitar duplicados en la unión.
    path_objetivo_invertido = flipud(coords_objetivo(2:end, :));

    % Concatena ambas coordenadas para formar el camino final.
    path_completo = [coords_inicio; path_objetivo_invertido];

    % Define el grosor y estilo según si es la ruta inicial o la optimizada.
    if strcmp(estilo, 'inicial')
        ancho = 2;
        color = 'r-'; % Rojo continuo
    else
        ancho = 2.5; % Un poco más gruesa para la óptima
        color = 'r-';
    end

    % Dibuja la trayectoria.
    plot(path_completo(:,1), path_completo(:,2), color, 'LineWidth', ancho);
    % Remarca los nodos individuales.
    plot(path_completo(:,1), path_completo(:,2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'black');
end
