function Exportar_Reporte_Metricas_V5(path, t_busqueda, n_total, n_camino, n_iter, n_col)
    costo = 0;
    for k = 1:size(path, 1) - 1
        costo = costo + norm(path(k+1, :) - path(k, :));
    end
    ruta_destino = fullfile(fileparts(mfilename('fullpath')), '..', 'Resultados_Metricas', ['Metricas_V5_', datestr(now, 'HHMMSS'), '.txt']);
    fid = fopen(ruta_destino, 'w');
    if fid ~= -1
        fprintf(fid, '=== REPORTE RRT BUSQUEDA BINARIA V5 ===\n');
        fprintf(fid, 'Tiempo Busqueda: %.4f s\nCosto Ruta: %.2f\nIteraciones: %d\nColisiones: %d\n', t_busqueda, costo, n_iter, n_col);
        fclose(fid);
        fprintf('Reporte V5 generado.\n');
    end
end
