% Exportar_Reporte_Metricas.m
% =========================================================================
% === FUNCION PARA CALCULAR COSTO Y GENERAR ARCHIVO DE RESULTADOS ===
% =========================================================================

function Exportar_Reporte_Metricas(path_final, t_busqueda, t_optimizacion, n_total_arboles, n_camino, n_iter, n_colisiones, costo_inicial)    % 1. Calcular la longitud (costo) total de la ruta definitiva.
    costo_final = 0;
    for k = 1:size(path_final, 1) - 1
        costo_final = costo_final + norm(path_final(k+1, :) - path_final(k, :));
    end

    % 2. Configurar rutas de guardado.
    ruta_del_script = fileparts(mfilename('fullpath'));
    nombre_carpeta = fullfile(ruta_del_script, '..', 'Resultados_Metricas');

    if ~exist(nombre_carpeta, 'dir')
        mkdir(nombre_carpeta);
    end

    nombre_archivo = ['Metricas_V24_', datestr(now, 'HHMMSS'), '.txt'];
    ruta_destino = fullfile(nombre_carpeta, nombre_archivo);

    % 3. Escribir el archivo.
    fid = fopen(ruta_destino, 'w');
    if fid ~= -1
        fprintf(fid, '=== REPORTE DE CALIDAD RRT* V24 ===\n');
        fprintf(fid, 'Tiempo Búsqueda Inicial: %.4f s\n', t_busqueda);
        fprintf(fid, 'Nodos totales en árboles: %d\n', n_total_arboles);
        fprintf(fid, 'Tiempo Optimización: %.4f s\n', t_optimizacion);
        fprintf(fid, 'Nodos en camino final: %d\n', n_camino);
        fprintf(fid, 'Costo Inicial de Ruta: %.2f\n', costo_inicial);
        fprintf(fid, 'Costo Final de Ruta: %.2f\n', costo_final);
        fprintf(fid, 'Iteraciones: %d | Colisiones: %d\n', n_iter, n_colisiones);

        eficiencia = (1 - (n_colisiones/n_iter)) * 100;
        fprintf(fid, 'Eficiencia de Muestreo: %.2f%%\n', eficiencia);
        fclose(fid);
        fprintf('\nReporte generado con éxito en: %s\n', ruta_destino);
    end
end
