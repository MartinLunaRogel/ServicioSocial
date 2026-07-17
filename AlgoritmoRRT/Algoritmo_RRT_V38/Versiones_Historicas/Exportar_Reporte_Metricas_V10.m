function Exportar_Reporte_Metricas_V10(path, t_busqueda, t_opt, n_total, n_camino, n_iter, n_col)
    costo = 0;
    for k = 1:size(path, 1) - 1
        costo = costo + norm(path(k+1, :) - path(k, :));
    end
    ruta_destino = fullfile(fileparts(mfilename('fullpath')), '..', 'Resultados_Metricas', ['Metricas_V10_', datestr(now, 'HHMMSS'), '.txt']);
    fid = fopen(ruta_destino, 'w');
    if fid ~= -1
        fprintf(fid, '=== REPORTE RRT VORAZ V10 ===\n');
        fprintf(fid, 'Tiempo Busqueda: %.4f s\nCosto Ruta: %.2f\nIteraciones: %d\nColisiones: %d\n', t_busqueda, costo, n_iter, n_col);
        fclose(fid);
        fprintf('Reporte V10 generado.\n');
    end
end
