function rand_point = Generar_Punto_Aleatorio(nodo_objetivo, current_nodes, x_limites, y_limites, coordenadas_esqueleto, mapa_discreto)
    r_prob = rand;

    % 1. Instinto Asesino (10%): Tirar directo al objetivo para forzar la conexión
    if r_prob < 0.10
        rand_point = nodo_objetivo;

    % 2. NUEVO: Instinto de Escape Local (25%)
    % Permite al árbol "reptar" fuera de rincones escondidos tirando puntos cerca de sí mismo
    elseif r_prob < 0.35
        punto_valido = false;
        while ~punto_valido
            % Elegimos un nodo que el árbol YA haya descubierto
            idx_nodo = randi(size(current_nodes, 1));
            nodo_base = current_nodes(idx_nodo, :);

            % Le sumamos un ruido aleatorio (Radio de 40 unidades para explorar cerca)
            ruido_x = (rand - 0.5) * 40;
            ruido_y = (rand - 0.5) * 40;

            rand_x = max(x_limites(1), min(nodo_base(1) + ruido_x, x_limites(2)));
            rand_y = max(y_limites(1), min(nodo_base(2) + ruido_y, y_limites(2)));

            % Confirmamos en O(1) que este nuevo punto esté en la calle
            idx_x = round(rand_x) + 1;
            idx_y = round(rand_y) + 1;

            if mapa_discreto(idx_y, idx_x) == 0
                rand_point = [rand_x, rand_y];
                punto_valido = true;
            end
        end

    % 3. Instinto Perfecto (65%): Crecer a lo largo de las avenidas principales
    else
        idx = randi(size(coordenadas_esqueleto, 1));
        rand_point = coordenadas_esqueleto(idx, :);
    end
end
