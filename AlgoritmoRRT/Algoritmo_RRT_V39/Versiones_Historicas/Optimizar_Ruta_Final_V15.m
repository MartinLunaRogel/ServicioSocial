function new_parent = Optimizar_Ruta_Final_V15(ax, nodes, parent, path_indices, obstaculos, Verificar_Colision_Segmento, distancia_muestreo)
    new_parent = parent;

    for i = 2:length(path_indices)
        idx_hijo = path_indices(i);
        hijo = nodes(idx_hijo, :);

        current_cost_to_hijo = Calcular_Costo_De_Ruta_V15(idx_hijo, nodes, new_parent);

        for idx_padre_candidato = 1:size(nodes, 1)
            if idx_padre_candidato == idx_hijo || idx_padre_candidato == 1
                continue;
            end

            padre_candidato = nodes(idx_padre_candidato, :);
            dist_padre_a_hijo = norm(hijo - padre_candidato);

            costo_to_candidato = Calcular_Costo_De_Ruta_V15(idx_padre_candidato, nodes, new_parent);
            potential_cost = costo_to_candidato + dist_padre_a_hijo;

            if potential_cost < current_cost_to_hijo
                if ~Verificar_Colision_Segmento(padre_candidato, hijo, obstaculos, distancia_muestreo)
                    plot(ax, [padre_candidato(1), hijo(1)], [padre_candidato(2), hijo(2)], 'g--', 'LineWidth', 1.5);
                    new_parent(idx_hijo) = idx_padre_candidato;
                    current_cost_to_hijo = potential_cost;
                    drawnow;
                end
            end
        end
    end
end
