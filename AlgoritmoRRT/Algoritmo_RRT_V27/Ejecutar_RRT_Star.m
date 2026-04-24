
% Algoritmo RRT con un primer obstaculos
% Martin Luna Rogel%
%
% Descripción:
%   Algoritmo RRT Bidireccional (Rapidly-exploring Random Tree).
%   El algoritmo genera dos árboles (uno desde inicio, otro desde objetivo) que crecen
%   simultáneamente hasta encontrarse en un "nodo de unión", buscando un camino óptimo.
%   Se añadioeron varios obstaculos cuadrados por todo el area, para que trate de rodearlos y encontrar el punto final
%
% Entradas:
%   - Ninguna.
%
% Salidas:
%   - Visualización grafica:
%       • Punto inicial (verde).
%       • Punto final (rojo).
%       • Expansión del árbol (líneas azules).
%       • Camino final encontrado (línea roja ).
%       • Obstaculo (barra al cento de la grafica)
pkg load statistics;
pkg load geometry;

% Al inicio de Ejecutar_RRT_Star.m
global obstaculos punto_inicio punto_final;

% Usamos los puntos del mouse
inicio = punto_inicio;
objetivo = punto_final;

% =========================================================================
%  === PARAMETROS DEL ESPACIO Y DEL ALGORITMO ===
% =========================================================================
x_limites = [0, 500];   % Define la anchura del espacio de trabajo, de 0 a 100 en el eje X.
y_limites = [0, 500];   % Define la altura del espacio de trabajo, de 0 a 100 en el eje Y.
tolerancia = 25;         % Distancia máxima al objetivo para considerar que se ha alcanzado. (Ahora se usa como radio de conexión)
max_iter = 5000;        % Número máximo de intentos para encontrar un camino.
step_size = 24;        % La longitud de cada paso del algoritmo.
prob_objetivo = 0.10;    % Probabilidad de muestrear el punto objetivo (5%)
grid_size = 25;         % Tamaño de la celda de la cuadricula (10x10 unidades)
prob_bias = 0.12;        % Probabilidad del 12% de usar el muestreo por densidad
num_celdas_x = ceil((x_limites(2) - x_limites(1)) / grid_size);
num_celdas_y = ceil((y_limites(2) - y_limites(1)) / grid_size);


% =========================================================================
% === USAR ARCHIVO APARTE DONDE ESTAN DEFINIDOS LOS OBSTACULOS ===
% =========================================================================
run('Mapa_Obstaculos_RRT.m');


% =========================================================================
% === INICIALIZACIÓN DEL BOSQUE (MALLA ESTRATÉGICA) ===
% =========================================================================

disp('Generando malla estratégica de semillas en las calles...');

% Definimos cada cuántos píxeles/unidades queremos un punto de la malla
% (60 es un excelente balance para tu mapa de 500x500)
distancia_malla = 60;

% Generamos la cuadrícula de semillas
semillas = Generar_Semillas_Malla(x_limites, y_limites, obstaculos, distancia_malla);

% Calculamos automáticamente cuántos árboles necesitamos en total
num_semillas = size(semillas, 1);
num_arboles = 2 + num_semillas; % 1 Inicio + 1 Objetivo + Total de Semillas sobrevivientes

disp(['> Se plantaron ', num2str(num_semillas), ' semillas en las calles libres.']);
disp(['> Iniciando bosque con ', num2str(num_arboles), ' árboles simultáneos.']);

% Consolidar todos los puntos base (raíces de los árboles)
% Índice 1: Inicio | Índice 2: Objetivo | Índices 3 al N: Semillas
raices_bosque = [inicio; objetivo; semillas];

% Inicializar las estructuras de datos (Cell Arrays)
bosque_nodos = cell(num_arboles, 1);
bosque_padres = cell(num_arboles, 1);
mapa_densidad_global = zeros(num_celdas_y, num_celdas_x);

for k = 1:num_arboles
    bosque_nodos{k} = raices_bosque(k, :);
    bosque_padres{k} = 0;

    % Actualizar la densidad en la matriz única
    [c_idx, r_idx] = Mapeo_Densidad_Grid(raices_bosque(k, :), x_limites, y_limites, grid_size);
    mapa_densidad_global(r_idx, c_idx) = mapa_densidad_global(r_idx, c_idx) + 1;
end

% 4. Matriz de Adyacencia y Detalles de Conexión
% matriz_conexiones(i, j) será 1 si el árbol 'i' se conectó con el árbol 'j'
matriz_conexiones = zeros(num_arboles, num_arboles);

% detalles_conexiones{i, j} guardará [idx_en_arbol_i, idx_en_arbol_j] para saber por qué nodos se unieron
detalles_conexiones = cell(num_arboles, num_arboles);

% Variables de estado de la búsqueda
is_connected = false;
ruta_arboles = []; % Guardará la secuencia de árboles (ej. [1, 5, 8, 2])


% --- NUEVAS VARIABLES PARA EXPORTAR MÉTRICAS ---
nodos_colisionados = 0;     % Contador de intentos que chocaron con obstáculos.
iteraciones_totales = 0;    % Registra cuántas iteraciones se ejecutaron realmente.
% (Asegúrate de que is_connected esté declarada aquí también)

% =========================================================================
% === AJUSTE DE LA DISTANCIA DE MUESTREO PARA POLÍGONOS ===
% =========================================================================
% Se establece un valor fijo bajo para garantizar la detección de colisiones
% en pasajes estrechos con obstáculos poligonales.
distancia_muestreo = 1.0; % Define una distancia de muestreo baja (0.5 unidades) para la verificación de colisión


% =========================================================================
%  === DIBUJAR EN EL ESPACIO DE TRABAJO (MODULARIZADO) ===
% =========================================================================

Inicializar_Graficos_RRT(x_limites, y_limites, inicio, objetivo, obstaculos);

% --- Dibujar las semillas en el mapa ---
plot(semillas(:,1), semillas(:,2), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 6);


% =========================================================================
%  === ALGORITMO RRT BOSQUE ===
% =========================================================================

% >>> INICIO DEL CONTEO DE TIEMPO PARA LA FASE DE BÚSQUEDA INICIAL <<<
tic;

for i = 1:max_iter
    iteraciones_totales = i;

    % --------------------------------------------------------------------------
    % --- 1. Lógica de Turnos (Round-Robin para 12 árboles) ---
    % --------------------------------------------------------------------------
    % mod(i-1, num_arboles) + 1 nos da una secuencia repetitiva: 1, 2, 3... 12, 1, 2...
    tree_id = mod(i-1, num_arboles) + 1;

    current_nodes = bosque_nodos{tree_id};
    current_parent = bosque_padres{tree_id};

    % Para el muestreo, haremos que todos los árboles tengan un ligero bias
    % hacia el objetivo final (índice 2) para "jalarlos" hacia la meta.
    nodo_meta_bias = raices_bosque(2, :);

    % --------------------------------------------------------------------------
    % --- 2. Expansión del Árbol Actual ---
    % --------------------------------------------------------------------------
    rand_point = Generar_Punto_Aleatorio(nodo_meta_bias, mapa_densidad_global, x_limites, y_limites, grid_size, prob_objetivo, prob_bias, semillas);

    dif = current_nodes - rand_point;
    distancias_sq = sum(dif.^2, 2);
    [~, idx] = min(distancias_sq);
    nearest_node = current_nodes(idx, :);

    direction = (rand_point - nearest_node);
    direction = direction / norm(direction);

    %Paso dinámico (Retracción en caso de choque)
    paso_actual = step_size;
    nodo_valido = false;

    while paso_actual >= 3 % Límite mínimo para no crear nodos inútilmente pegados

      new_node = nearest_node + paso_actual * direction;

      if ~Verificar_Colision_Segmento(nearest_node, new_node, obstaculos, distancia_muestreo)
        nodo_valido = true;
        break; % Encontramos un paso libre, salimos del while
      else
        paso_actual = paso_actual / 2; % Chocó. Reducimos el paso a la mitad y reintentamos.
      end
    end

    if nodo_valido
        % Añadir el nodo al árbol actual
        current_nodes = [current_nodes; new_node];
        current_parent(end+1) = idx;
        new_node_idx = size(current_nodes, 1);

        % Actualizar la densidad en la MATRIZ GLOBAL
        [c_idx, r_idx] = Mapeo_Densidad_Grid(new_node, x_limites, y_limites, grid_size);
        mapa_densidad_global(r_idx, c_idx) = mapa_densidad_global(r_idx, c_idx) + 1;

        % Guardar cambios en el bosque
        bosque_nodos{tree_id} = current_nodes;
        bosque_padres{tree_id} = current_parent;

        % Dibujar la rama (Azul para inicio, Cyan para objetivo, Verde para semillas)
        if tree_id == 1
            color_rama = 'b';
        elseif tree_id == 2
            color_rama = 'c';
        else
            color_rama = 'g';
        end
        plot([nearest_node(1), new_node(1)], [nearest_node(2), new_node(2)], color_rama);

        %% Dubujar solo cada 5 iteraciones (para acelerar el programa)
        if mod(i, 100) == 0
            drawnow;
        end


        % --------------------------------------------------------------------------
        % --- 3. INTENTO DE CONEXIÓN CON EL RESTO DEL BOSQUE ---
        % --------------------------------------------------------------------------
        for otro_id = 1:num_arboles
            % No conectarse a sí mismo, ni a árboles con los que YA está conectado
            if otro_id == tree_id || matriz_conexiones(tree_id, otro_id) == 1
                continue;
            end

            other_nodes = bosque_nodos{otro_id};
            dif_other = other_nodes - new_node;
            distancias_other_sq = sum(dif_other.^2, 2);
            [min_dist_sq, idx_nearest_other] = min(distancias_other_sq);

            % --- OPTIMIZACIÓN: Solo intentar conectar si están en el vecindario ---
            % Esto evita llamar a la función pesada de colisiones para nodos lejanos
            if min_dist_sq < (tolerancia^2)
                nodo_candidato = other_nodes(idx_nearest_other, :);

                % Solo si están cerca, verificamos si hay una pared en medio
                if ~Verificar_Colision_Segmento(new_node, nodo_candidato, obstaculos, distancia_muestreo)

                    % ¡SE DIERON LA MANO! (Registramos la conexión)
                    matriz_conexiones(tree_id, otro_id) = 1;
                    matriz_conexiones(otro_id, tree_id) = 1;

                    % Guardamos EXACTAMENTE qué nodos se unieron
                    detalles_conexiones{tree_id, otro_id} = [new_node_idx, idx_nearest_other];
                    detalles_conexiones{otro_id, tree_id} = [idx_nearest_other, new_node_idx];

                    plot([new_node(1), other_nodes(idx_nearest_other, 1)], [new_node(2), other_nodes(idx_nearest_other, 2)], 'm', 'LineWidth', 1);

                    %% Solamente dibujar cada 5 iteraciones para acelerar el programa
                    if mod(i, 100) == 0
                        drawnow;
                    end
                    disp(['> Conexión formada entre el árbol ', num2str(tree_id), ' y el árbol ', num2str(otro_id)]);

                    % ------------------------------------------------------------------
                    % --- 4. VERIFICAR SI LA RED YA CONECTÓ EL INICIO (1) Y OBJETIVO (2)
                    % ------------------------------------------------------------------

                    % Antes de llamar a BFS, asegúrate de que el Inicio (1) y el Objetivo (2)
                    % tengan al menos una conexión en la matriz. Si están en ceros, es imposible que haya ruta.
                    if sum(matriz_conexiones(1,:)) > 0 && sum(matriz_conexiones(2,:)) > 0
                        ruta_arboles = BFS_Encontrar_Ruta(matriz_conexiones, 1, 2);
                        if ~isempty(ruta_arboles)
                          is_connected = true; % Usamos tu misma variable original
                          disp('¡RED COMPLETADA! Existe un camino continuo desde el Inicio hasta el Objetivo.');
                          disp(['Secuencia de árboles conectados: ', num2str(ruta_arboles)]);
                          break; % Rompe el for de conexiones
                        end
                    end

                end
            end
        end % Fin for conexiones

        if is_connected
            break; % Rompe el bucle principal for max_iter
        end
    else
        nodos_colisionados = nodos_colisionados + 1;
    end
end

% >>> FIN DEL CONTEO DE TIEMPO PARA LA BÚSQUEDA INICIAL <<<
tiempo_busqueda_inicial = toc;


if is_connected
    disp('Unificando bosque y extrayendo ruta inicial...');

    % ----------------------------------------------------------------------
    % === 1. UNIFICAR BOSQUE Y RECONSTRUIR CAMINO INICIAL ===
    % ----------------------------------------------------------------------
    [nodes_global, parent_global, path_coords_inicial, path_indices_global] = Unificar_Bosque_RRT(bosque_nodos, bosque_padres, detalles_conexiones, num_arboles);

    total_nodos_encontrados = size(nodes_global, 1);

    % ----------------------------------------------------------------------
    % === 2. DIBUJAR CAMINO INICIAL COMPLETO ===
    % ----------------------------------------------------------------------
    Limpiar_Dibujos_RRT('union');

    % Dibujamos la ruta inicial en rojo continuo (ancho 2)
    plot(path_coords_inicial(:,1), path_coords_inicial(:,2), 'r-', 'LineWidth', 2);
    plot(path_coords_inicial(:,1), path_coords_inicial(:,2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'black');
    drawnow;

    % --- CALCULAR COSTO DE LA RUTA INICIAL ---
    costo_inicial = 0;
    for k = 1:size(path_coords_inicial, 1) - 1
        costo_inicial = costo_inicial + norm(path_coords_inicial(k+1, :) - path_coords_inicial(k, :));
    end

    % ----------------------------------------------------------------------
    % === FASE DE OPTIMIZACIÓN RRT* (POST-PROCESAMIENTO SHORTCUTTING) ===
    % ----------------------------------------------------------------------
    disp('Iniciando optimización de caminos (Suavizado por atajos)...');
    tic;

    % Le pasamos directamente las coordenadas iniciales, sin complicaciones
    path_final_coords_completo = Optimizar_Ruta_Final(path_coords_inicial, obstaculos, @Verificar_Colision_Segmento, distancia_muestreo);

    tiempo_optimizacion = toc;

    % ----------------------------------------------------------------------
    % === DIBUJAR EL CAMINO FINAL OPTIMIZADO ===
    % ----------------------------------------------------------------------
    Limpiar_Dibujos_RRT('fase_optimizacion');

    % Dibujar ruta final óptima (línea roja gruesa)
    plot(path_final_coords_completo(:,1), path_final_coords_completo(:,2), 'r-', 'LineWidth', 2.5);
    plot(path_final_coords_completo(:,1), path_final_coords_completo(:,2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'black');
    drawnow;

    nodos_camino_final = size(path_final_coords_completo, 1);
    disp('Optimización finalizada. Camino más corto trazado.');

    % ==================================================================
    % === EXPORTACIÓN A CARPETA DE RESULTADOS ===
    % ==================================================================
    Exportar_Reporte_Metricas(path_final_coords_completo, tiempo_busqueda_inicial, tiempo_optimizacion, total_nodos_encontrados, nodos_camino_final, iteraciones_totales, nodos_colisionados, costo_inicial);
else
    disp('No se encontró conexión dentro del número máximo de iteraciones.');
end
