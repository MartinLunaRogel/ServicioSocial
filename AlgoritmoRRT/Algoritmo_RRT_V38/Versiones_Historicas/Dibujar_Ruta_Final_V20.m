function path_completo = Dibujar_Ruta_Final_V20(ax, coords_inicio, coords_objetivo, estilo)
    path_objetivo_invertido = flipud(coords_objetivo(2:end, :));
    path_completo = [coords_inicio; path_objetivo_invertido];

    if strcmp(estilo, 'inicial')
        ancho = 2; color = 'r-';
    else
        ancho = 2.5; color = 'r-';
    end

    plot(ax, path_completo(:,1), path_completo(:,2), color, 'LineWidth', ancho);
    plot(ax, path_completo(:,1), path_completo(:,2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'black');
end
