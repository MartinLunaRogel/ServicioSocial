function Limpiar_Dibujos_V20(ax, tipo)
    if strcmp(tipo, 'union')
        delete(findobj(ax, 'Type', 'line', 'Color', 'm'));
    elseif strcmp(tipo, 'fase_optimizacion')
        delete(findobj(ax, 'Type', 'line', 'LineWidth', 2));
        delete(findobj(ax, 'Type', 'line', 'MarkerSize', 4));
        delete(findobj(ax, 'Type', 'line', 'Color', 'g', 'LineWidth', 1.5));
    end
end
