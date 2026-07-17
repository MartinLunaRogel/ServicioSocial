function hay_colision = Verificar_Colision_Segmento_V15(node1, node2, obstaculos, distancia_muestreo)
    distancia_total = norm(node2 - node1);
    num_subdivisiones = ceil(distancia_total / distancia_muestreo);

    if num_subdivisiones == 0
        num_subdivisiones = 1;
    end

    puntos_de_prueba_x = zeros(num_subdivisiones + 1, 1);
    puntos_de_prueba_y = zeros(num_subdivisiones + 1, 1);

    for j = 0:num_subdivisiones
        punto_intermedio = node1 + (j / num_subdivisiones) * (node2 - node1);
        puntos_de_prueba_x(j+1) = punto_intermedio(1);
        puntos_de_prueba_y(j+1) = punto_intermedio(2);
    end

    for i = 1:size(obstaculos, 1)
        vertices = obstaculos{i, 1};
        V_x = vertices(:, 1);
        V_y = vertices(:, 2);

        in = inpolygon(puntos_de_prueba_x, puntos_de_prueba_y, V_x, V_y);

        if any(in)
            hay_colision = true;
            return;
        end
    end

    hay_colision = false;
end
