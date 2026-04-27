% Le pasamos mapa_discreto en lugar de obstaculos
function path_final_coords = Optimizar_Ruta_Final(path_coords, mapa_discreto, Verificar_Colision_Segmento)
    path_optimizado = path_coords;
    max_intentos = 400;

    for k = 1:max_intentos
        num_nodos = size(path_optimizado, 1);
        if num_nodos <= 2
            break;
        end

        idx1 = randi(num_nodos - 2);
        idx2 = randi([idx1 + 2, num_nodos]);

        nodo_A = path_optimizado(idx1, :);
        nodo_B = path_optimizado(idx2, :);

        % Nueva llamada súper rápida a la verificación
        if ~Verificar_Colision_Segmento(nodo_A, nodo_B, mapa_discreto)
            path_optimizado(idx1+1 : idx2-1, :) = [];
        end
    end
    path_final_coords = path_optimizado;
end
