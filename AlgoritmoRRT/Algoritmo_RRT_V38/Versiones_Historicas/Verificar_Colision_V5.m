function hay_colision = Verificar_Colision_V5(node1, node2, obstaculos)
    % Muestreo del segmento
    distancia_total = norm(node2 - node1);
    num_sub = ceil(distancia_total / 2.5); % Distancia de muestreo fija
    if num_sub == 0, num_sub = 1; end

    for j = 0:num_sub
        p = node1 + (j / num_sub) * (node2 - node1);
        for i = 1:size(obstaculos, 1)
            % Comprobación contra polígonos (patches de OSM)
            if inpolygon(p(1), p(2), obstaculos{i,1}(:,1), obstaculos{i,1}(:,2))
                hay_colision = true; return;
            end
        end
    end
    hay_colision = false;
end
