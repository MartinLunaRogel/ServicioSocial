% =========================================================================
% === FUNCION DE FUSIÓN: UNIFICAR EL BOSQUE Y EXTRAER LA RUTA FINAL ===
%
% A lo largo de la búsqueda, creamos varios árboles separados y cuando esos árboles por fin chocan y se unen, esta función entra
% en acción. Toma todas las ramas dispersas, las pega en un solo "Mega-Árbol" y luego rastrea exactamente qué ramas debemos seguir
% para ir desde el punto A (Inicio) hasta el punto B (Objetivo).
% =========================================================================
function [nodes_global, parent_global, path_coords, path_indices] = Unificar_Bosque_RRT(bosque_nodos, bosque_padres, detalles_conexiones, num_arboles)

    % --- CALCULAR LOS "DESPLAZAMIENTOS" ---
    offsets = zeros(1, num_arboles);                          % Creamos una lista de ceros para guardar dónde empieza cada árbol
    total_nodos = 0;                                          % Iniciamos un contador para saber cuántos puntos (nodos) llevamos en total
    for k = 1:num_arboles                                     % Revisamos árbol por árbol
        offsets(k) = total_nodos;                             % Anotamos que el árbol actual empezará después del total de nodos que ya llevamos
        total_nodos = total_nodos + size(bosque_nodos{k}, 1); % Sumamos al contador la cantidad de puntos (ramas) que tiene este árbol en específico
    end

    % --- JUNTAR TODAS LAS COORDENADAS EN UNA LISTA ---

    nodes_global = zeros(total_nodos, 2);                     % Preparamos una tabla de ceros con 2 columnas (X, Y) y filas suficientes para todos los puntos
    for k = 1:num_arboles                                     % Volvemos a recorrer árbol por árbol

        idx_start = offsets(k) + 1;                           % Calculamos el renglón de inicio: el offset de este árbol + 1
        idx_end = offsets(k) + size(bosque_nodos{k}, 1);      % Calculamos el renglón final: el inicio + la cantidad de puntos que tiene el árbol
        nodes_global(idx_start:idx_end, :) = bosque_nodos{k}; % Pegamos las coordenadas (X, Y) de este árbol en su espacio correspondiente de la lista
    end

    % --- DIBUJAR EL MAPA DE PARENTESCO (QUIÉN ESTÁ CONECTADO CON QUIÉN) ---

    adyacencia = cell(total_nodos, 1);                        % Preparamos una lista vacía para cada punto existente. Aquí anotaremos sus vecinos.
    for k = 1:num_arboles                                     % Recorremos cada árbol para anotar sus conexiones internas (ramas de un mismo árbol)
        padres = bosque_padres{k};                            % Sacamos la lista de "padres" (quién dio a luz a quién) de este árbol

        for i = 2:length(padres)                              % Empezamos a revisar desde el punto 2 (porque el punto 1 es la raíz y no tiene padre)
            hijo_global = offsets(k) + i;                     % Traducimos el número de hijo a su nueva posición en la lista
            padre_global = offsets(k) + padres(i);            % Traducimos el número del padre a su nueva posición en la lista gigante
            adyacencia{hijo_global} = [adyacencia{hijo_global}, padre_global];  % Anotamos que el hijo está conectado con el padre
            adyacencia{padre_global} = [adyacencia{padre_global}, hijo_global]; % Anotamos de regreso que el padre también está conectado con el hijo (camino de doble vía)
        end
    end

    % --- CONECTAR LOS PUENTES (CUANDO DOS ÁRBOLES DISTINTOS SE DIERON LA MANO) ---

    for i = 1:num_arboles                                     % Comparamos todos los árboles contra todos los demás para buscar dónde se tocaron
        for j = i+1:num_arboles

            if ~isempty(detalles_conexiones{i, j})            % Si la casilla de detalles no está vacía, significa que aquí hay un puente

                nodo_en_i = offsets(i) + detalles_conexiones{i, j}(1);          % Traducimos el punto del árbol 'i' que formó el puente a su número global
                nodo_en_j = offsets(j) + detalles_conexiones{i, j}(2);          % Traducimos el punto del árbol 'j' que formó el puente a su número global
                adyacencia{nodo_en_i} = [adyacencia{nodo_en_i}, nodo_en_j];     % Anotamos en nuestro mapa que estos dos puntos de árboles distintos ahora están conectados
                adyacencia{nodo_en_j} = [adyacencia{nodo_en_j}, nodo_en_i];
            end
        end
    end

    % --- RASTREAR LA RUTA FINAL (ALGORITMO DE BÚSQUEDA) ---

    idx_inicio_global = offsets(1) + 1;                       % Sabemos que el inicio (Punto A) siempre es la raíz del primer árbol
    idx_objetivo_global = offsets(2) + 1;                     % Sabemos que la meta (Punto B) siempre es la raíz del segundo árbol
    parent_global = zeros(total_nodos, 1);                    % Preparamos una lista de ceros para guardar la ruta paso a paso
    visitado = false(total_nodos, 1);                         % Preparamos una lista de apagadores para no revisar el mismo punto dos veces
    cola = idx_inicio_global;                                 % Empezamos la búsqueda formándonos en el punto de inicio
    visitado(idx_inicio_global) = true;                       % Marcamos el inicio como ya visitado

    while ~isempty(cola)                                      % Mientras siga habiendo puntos en la fila de búsqueda...

        actual = cola(1);                                     % Tomamos el primer punto de la fila
        cola(1) = [];                                         % Lo borramos de la fila de espera

        if actual == idx_objetivo_global                      % Si este punto actual es la Meta que estamos buscando
            break;                                            % Se rompe el ciclo por que ya se encontro conexion
        end

        vecinos = adyacencia{actual};                         % Si no es la meta, revisamos quiénes son los vecinos conectados a este punto

        for v = vecinos                                       % Por cada vecino conectado...
            if ~visitado(v)                                   % Entra si este punto no ha sido visitado antes

                visitado(v) = true;                           % Lo marcamos como visitado
                parent_global(v) = actual;                    % Anotamos que llegamos a este vecino gracias al punto 'actual' (trazamos el paso)
                cola = [cola, v];                             % Formamos a este vecino al final de la fila para revisarlo después
            end
        end
    end

    % --- EXTRAER LAS COORDENADAS DEL CAMINO FINAL ---

    path_indices = [];                                        % Preparamos una lista vacía para guardar la secuencia de pasos ganadora
    curr = idx_objetivo_global;                               % Nos paramos en la meta y vamos a empezar a caminar hacia atrás
    while curr ~= 0                                           % Mientras no lleguemos al vacío (el inicio tiene un 0 como padre)

        path_indices = [curr; path_indices];                  % Anotamos este paso al principio de nuestra lista de la ruta final
        curr = parent_global(curr);                           % Retrocedemos al punto anterior usando nuestra libreta de "padres"
    end

    path_coords = nodes_global(path_indices, :);              % traducimos esos "números de paso" a coordenadas (X, Y) reales usando nuestra lista
end
