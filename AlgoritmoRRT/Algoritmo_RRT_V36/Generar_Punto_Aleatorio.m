% =========================================================================
% === FUNCION DE INTELIGENCIA: GENERACIÓN DE PUNTOS ALEATORIOS ===
%
% función que de decide hacia dónde va a crecer el árbol azul en cada turno. E
% utiliza tres "instintos" (probabilidades) para ser más inteligente:
% intentar conectar con la meta, buscar atajos locales o crecer sobre las calles principales.
% =========================================================================
function rand_point = Generar_Punto_Aleatorio(nodo_objetivo, current_nodes, x_limites, y_limites, coordenadas_esqueleto, mapa_discreto)

    % Generamos un número aleatorio entre 0.0 y 1.0 (como tirar unos dados de 100 caras)
    % Esto nos servirá para elegir qué "instinto" usará el árbol en este turno exacto
    r_prob = rand;

    % =====================================================================
    % 1. Forzar al árbol a intentar conectarse directamente con la meta (10% de probabilidad).
    % =====================================================================
    if r_prob < 0.10

        % Ignora todo y tira un punto exactamente en las coordenadas de la meta
        rand_point = nodo_objetivo;

    % =====================================================================
    % Si el árbol se atora en un callejón o esquina, le permite "reptar" tirando puntos muy cerquita de sí mismo para rodear poco a poco
    % Tiene un 25 de probabilidad
    % =====================================================================
    elseif r_prob < 0.35

        % Creamos un interruptor apagado: seguiremos buscando hasta hallar un punto que caiga en la calle
        punto_valido = false;

        % Mientras no encontremos un punto válido (que no esté dentro de un obstaculo)
        while ~punto_valido

            % Elegimos al azar un(nodo) que el árbol ya haya descubierto antes
            idx_nodo = randi(size(current_nodes, 1));

            % Extraemos las coordenadas (X, Y) de ese nodo que elegimos
            nodo_base = current_nodes(idx_nodo, :);

            % Le sumamos un "ruido" aleatorio: un número entre -20 y +20 unidades
            % Esto crea un círculo de exploración pequeño alrededor del nodo
            ruido_x = (rand - 0.5) * 40;
            ruido_y = (rand - 0.5) * 40;

            % Calculamos el nuevo punto en X, asegurándonos de que no se salga de los bordes del mapa
            rand_x = max(x_limites(1), min(nodo_base(1) + ruido_x, x_limites(2)));

            % Calculamos el nuevo punto en Y, asegurándonos también de no salirnos del mapa
            rand_y = max(y_limites(1), min(nodo_base(2) + ruido_y, y_limites(2)));

            % Convertimos las coordenadas (X, Y) reales a números enteros para poder buscarlos en la matriz
            idx_x = round(rand_x) + 1;
            idx_y = round(rand_y) + 1;

            % Consultamos nuestra matriz de edificios (mapa_discreto).
            % Si en esa fila y columna hay un 0, significa que es calle libre.
            if mapa_discreto(idx_y, idx_x) == 0

                % Encontramos un punto válido en la calle. Lo guardamos.
                rand_point = [rand_x, rand_y];

                % Encendemos el interruptor para romper el ciclo 'while' y salir de la función
                punto_valido = true;
            end
        end

    % =====================================================================
    % Es la acción por defecto. Hace que el árbol crezca rápida y naturalmente a lo largo de las calles del mapa usando el esqueleto generado
    % Actua un 65% de veces
    % =====================================================================
    else
        % Elegimos un número de fila al azar de toda nuestra enorme lista de calles libres (coordenadas_esqueleto)
        idx = randi(size(coordenadas_esqueleto, 1));

        % Tomamos la coordenada exacta (X, Y) que estaba en esa fila elegida y se la damos al árbol
        rand_point = coordenadas_esqueleto(idx, :);
    end
end
