% =========================================================================
% === FUNCION DE REPORTE: EXPORTACIÓN DE MÉTRICAS ===
%
% Se calcula la distancia final de la ruta y se genera un archivo de texto (.txt) con todas las
% estadísticas de rendimiento de la búsqueda para su posterior análisis.
% =========================================================================
function Exportar_Reporte_Metricas(path_final, t_busqueda, t_optimizacion, n_total_arboles, n_camino, n_iter, n_colisiones, costo_inicial)

    % --- CÁLCULO DE LA DISTANCIA FINAL ---

    costo_final = 0;                                                    % Se inicializa el contador de distancia total en cero.
    for k = 1:size(path_final, 1) - 1                                   % Se recorren todos los puntos de la ruta final, excepto el último.

        costo_final = costo_final + norm(path_final(k+1, :) - path_final(k, :));% Se suma la distancia matemática (norma) entre el punto actual y el siguiente.
    end

    % --- CONFIGURACIÓN DEL ARCHIVO DE GUARDADO ---

    ruta_del_script = fileparts(mfilename('fullpath'));                 % Se obtiene la ubicación actual de este script en la computadora.
    nombre_carpeta = fullfile(ruta_del_script, '..', 'Resultados_Metricas');% Se define la ruta de una nueva carpeta llamada 'Resultados_Metricas'.

    if ~exist(nombre_carpeta, 'dir')                                    % Se verifica si dicha carpeta no existe aún en el sistema.
        mkdir(nombre_carpeta);                                          % De no existir, se crea la carpeta.
    end

    nombre_archivo = ['Metricas_V35_', datestr(now, 'HHMMSS'), '.txt']; % Se genera un nombre único para el archivo usando la versión y la hora actual.
    ruta_destino = fullfile(nombre_carpeta, nombre_archivo);            % Se construye la ruta completa uniendo la carpeta y el nombre del archivo.

    % --- ESCRITURA DE DATOS ---

    fid = fopen(ruta_destino, 'w');                                     % Se crea y se abre el archivo en modo escritura ('w').
    if fid ~= -1                                                        % Se verifica que el archivo se haya abierto correctamente sin errores.

        % Se imprimen los títulos y los datos básicos de tiempo y nodos.
        fprintf(fid, '=== REPORTE DE CALIDAD RRT* V35 ===\n');
        fprintf(fid, 'Tiempo Búsqueda Inicial: %.4f s\n', t_busqueda);
        fprintf(fid, 'Nodos totales en árboles: %d\n', n_total_arboles);
        fprintf(fid, 'Tiempo Optimización: %.4f s\n', t_optimizacion);
        fprintf(fid, 'Nodos en camino final: %d\n', n_camino);

        % Se imprimen los costos de la ruta antes y después de optimizarla.
        fprintf(fid, 'Costo Inicial de Ruta: %.2f\n', costo_inicial);
        fprintf(fid, 'Costo Final de Ruta: %.2f\n', costo_final);

        % Se imprime el recuento de intentos y fallos.
        fprintf(fid, 'Iteraciones: %d | Colisiones: %d\n', n_iter, n_colisiones);

        % Se calcula el porcentaje de éxito al elegir puntos libres de obstáculos.
        eficiencia = (1 - (n_colisiones/n_iter)) * 100;

        % Se imprime la eficiencia calculada en el archivo.
        fprintf(fid, 'Eficiencia de Muestreo: %.2f%%\n', eficiencia);

        % Se cierra y se guarda el archivo en el disco duro.
        fclose(fid);

        % Se muestra un mensaje de confirmación en la consola de Octave.
        fprintf('\nReporte generado con éxito en: %s\n', ruta_destino);
    end
end
