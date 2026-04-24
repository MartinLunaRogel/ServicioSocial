% Limpiar_Dibujos_RRT.m
% =========================================================================
% === FUNCION PARA BORRAR ELEMENTOS GRAFICOS ESPECIFICOS ===
% =========================================================================

function Limpiar_Dibujos_RRT(tipo)
    if strcmp(tipo, 'union')
        % Borra la línea magenta de unión.
        delete(findobj(gca, 'Type', 'line', 'Color', 'm'));
    elseif strcmp(tipo, 'fase_optimizacion')
        % Borra las trayectorias iniciales (LineWidth 2).
        delete(findobj(gca, 'Type', 'line', 'LineWidth', 2));
        % Borra los marcadores previos (MarkerSize 4).
        delete(findobj(gca, 'Type', 'line', 'MarkerSize', 5));
        % Borra las líneas verdes temporales de rewiring.
        delete(findobj(gca, 'Type', 'line', 'Color', 'g', 'LineWidth', 1.5));
    end
end
