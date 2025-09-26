% ============================================================
% verificar_colisiones:
%   Revisa si el segmento entre el nodo mas cercano y el nuevo nodo colisiona con un obstáculo,
%   usando puntos intermedios.
%   La división de la línea se ajusta automáticamente para que cada sub-segmento
%   sea más pequeño que la dimensión más estrecha del obstáculo.

function hay_colision = verificar_colision(nodo_inicio, nodo_final, obstaculo_x, obstaculo_ancho, obstaculo_y)

    hay_colision = false; % Se inicializa la variable de colisión como falso

    % 1. Calcular los límites del obstáculo
    x_izquierda = obstaculo_x - obstaculo_ancho / 2;
    x_derecha   = obstaculo_x + obstaculo_ancho / 2;
    y_inferior  = obstaculo_y(1);
    y_superior  = obstaculo_y(2);

    % 2. Determinar la distancia de muestreo
    % La distancia entre cada sub-punto debe ser más pequeña que la dimensión más estrecha del obstáculo.
    % Se recomienda dividir el ancho del obstáculo por un factor (ej. 2 o 3) para mayor seguridad.
    distancia_muestreo = obstaculo_ancho / 3;
    if distancia_muestreo == 0
        distancia_muestreo = 0.1; % Evitar división por cero si el obstáculo tiene un ancho de cero
    end

    distancia_total = norm(nodo_final - nodo_inicio);   %se obtiene la distancia total entre el nuevo nodo y el nodo mas cercano
    num_subdivisiones = ceil(distancia_total / distancia_muestreo);   % se divide esa distancia total entre la distancia de muestreo, para obtener el numero de subdivisiones de la linea

    % 3. Muestrear a lo largo del segmento de línea
    for i = 0:num_subdivisiones
        punto_intermedio = nodo_inicio + (i / num_subdivisiones) * (nodo_final - nodo_inicio);  % Se calcula la posición de los puntos intermedios

        % Se verifica si el punto intermedio está dentro del obstáculo actual
        if (punto_intermedio(1) >= x_izquierda && punto_intermedio(1) <= x_derecha && ...
            punto_intermedio(2) >= y_inferior && punto_intermedio(2) <= y_superior)
            hay_colision = true; % Si está dentro, hay colisión
            return;               % Salir inmediatamente
        end
    end
end
