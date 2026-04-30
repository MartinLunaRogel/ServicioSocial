% =========================================================================
% === FUNCION DE LIMPIEZA VISUAL: BORRADO DE DIBUJOS TEMPORALES ===
%
% Durante ejecución, el algoritmo dibuja líneas para mostrar visualmente cómo se exploran las rutas. Esta función se encarga
% de borrar esos trazos temporales de la gráfica gráfica sin afectar el mapa de fondo ni los puntos de inicio y fin, manteniendo la pantalla limpia.
% =========================================================================
function Limpiar_Dibujos_RRT(tipo)

    if strcmp(tipo, 'union')                                % Se evalúa si el texto recibido en la variable 'tipo' es exactamente igual a la palabra 'union'.

        % --- LIMPIEZA DE LÍNEAS DE CONEXIÓN ---

        delete(findobj(gca, 'Type', 'line', 'Color', 'm')); % Se busca en los ejes actuales (gca) cualquier objeto de tipo línea que tenga el color magenta ('m').
                                                            % Una vez encontrados, se utiliza el comando 'delete' para eliminarlos de la pantalla.

    elseif strcmp(tipo, 'fase_optimizacion')                % Si el texto recibido no fue 'union', se evalúa si corresponde a la fase de optimización.

        % --- LIMPIEZA DE FASE DE OPTIMIZACIÓN ---

        delete(findobj(gca, 'Type', 'line', 'LineWidth', 2)); % Se buscan y se eliminan todas las líneas de la ruta inicial (identificadas por su grosor de 2).
        delete(findobj(gca, 'Type', 'line', 'MarkerSize', 5));% Se buscan y se eliminan todos los marcadores circulares de los vértices intermedios (tamaño 5).
        delete(findobj(gca, 'Type', 'line', 'Color', 'g', 'LineWidth', 1.5)); % Se eliminan las líneas verdes temporales utilizadas para mostrar el recableado de la ruta.
    end
end
