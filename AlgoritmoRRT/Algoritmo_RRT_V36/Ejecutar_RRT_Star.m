% =========================================================================
% === MOTOR CENTRAL: EJECUCIÓN DEL ALGORITMO RRT* BÚSQUEDA ===
%
% Es el nucleo del programa. Coordina el crecimiento simultáneo del árbol inicial, el árbol meta y las semillas.
% Evalúa colisiones, forma conexiones entre ramas y busca el camino más corto.
% =========================================================================

function Ejecutar_RRT_Star()
    % Llamada a las variables de la memoria RAM.
    global obstaculos punto_inicio punto_final mapa_esqueleto coordenadas_esqueleto mapa_discreto semillas;

    inicio = punto_inicio;                      % Asignación de las coordenadas de arranque.
    objetivo = punto_final;                     % Definición de la meta a alcanzar.

    % =========================================================================
    %  === PARAMETROS DEL ESPACIO Y DEL ALGORITMO ===
    % =========================================================================
    x_limites = [0, 500];                       % Límite horizontal del área de trabajo.
    y_limites = [0, 500];                       % Límite vertical del plano.
    tolerancia = 25;                            % Radio máximo para intentar conectar dos ramas cercanas.
    max_iter = 15000;                           % Tope de intentos para evitar ciclos infinitos.
    step_size = 24;                             % Longitud estándar de cada rama nueva.


    % =========================================================================
    % === INICIALIZACIÓN DEL BOSQUE Y CARGA DE MAPAS DIGITALES ===
    % =========================================================================

    num_semillas = size(semillas, 1);           % Conteo total de semillas disponibles.
    num_arboles = 2 + num_semillas;             % Suma del inicio, meta y todas las semillas.

    disp(['> Se plantaron ', num2str(num_semillas), ' semillas en las calles libres.']); % Notificación en consola.
    disp(['> Iniciando bosque con ', num2str(num_arboles), ' árboles simultáneos.']); % Confirmación de arranque.

    raices_bosque = [inicio; objetivo; semillas]; % Agrupación de todos los puntos de partida en una lista maestra.

    bosque_nodos = cell(num_arboles, 1);          % Creación de estructura vacía para almacenar las coordenadas de los árboles.
    bosque_padres = cell(num_arboles, 1);         % Estructura para registrar la genealogía (padres) de las ramas.

    for k = 1:num_arboles                         % Preparación inicial de cada árbol.
        bosque_nodos{k} = raices_bosque(k, :);    % Registro de la posición raíz del árbol.
        bosque_padres{k} = 0;                     % Asignación de cero a la raíz (sin padre).
    end

    matriz_conexiones = zeros(num_arboles, num_arboles);  % Tabla para rastrear qué árbol tocó a qué otro árbol.
    detalles_conexiones = cell(num_arboles, num_arboles); % Registro del punto exacto de choque entre árboles.

    is_connected = false;                         % Bandera inicial de éxito en estado inactivo.

    mejor_costo = inf;                            % Establecimiento del récord inicial en infinito.
    path_coords_inicial = [];                     % Almacenamiento temporal del primer camino detectado.
    h_mejor_ruta = [];                            % Referencia gráfica para la línea de la ruta principal.
    iteraciones_extra = 0;                        % Contador para el periodo de gracia posterior a la conexión.

    nodos_colisionados = 0;                       % Registro estadístico de choques contra paredes.
    iteraciones_totales = 0;                      % Conteo del esfuerzo real del algoritmo.


    % =========================================================================
    %  === DIBUJAR EN EL ESPACIO DE TRABAJO (MODULARIZADO) ===
    % =========================================================================

    plot(semillas(:,1), semillas(:,2), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 6); % Trazado visual de las semillas en amarillo.


    % =========================================================================
    %  === ALGORITMO RRT BOSQUE ===
    % =========================================================================

    tic;                                          % Arranque del cronómetro interno para la métrica de tiempo.

    for i = 1:max_iter                            % Inicio del ciclo de exploración repetitiva.
        iteraciones_totales = i;                  % Actualización del contador global de ciclos.

        tree_id = mod(i-1, num_arboles) + 1;      % Asignación de turnos equitativos (round-robin) para los árboles.

        if tree_id > 2 && sum(matriz_conexiones(tree_id, :)) >= 2   % Verificación de puentes activos en semillas.
            continue;                                               % Salto de turno para optimizar rendimiento si la semilla ya cumplió su labor.
        end

        current_nodes = bosque_nodos{tree_id};    % Extracción de ramas del árbol en turno.
        current_parent = bosque_padres{tree_id};  % Extracción del árbol genealógico actual.

        nodo_meta_bias = raices_bosque(2, :);     % Definición del objetivo principal como punto de gravedad.

        rand_point = Generar_Punto_Aleatorio(nodo_meta_bias, current_nodes, x_limites, y_limites, coordenadas_esqueleto, mapa_discreto); % Uso de inteligencia artificial para la siguiente dirección.

        dif = current_nodes - rand_point;         % Medición matemática de distancia hacia el punto aleatorio.
        distancias_sq = sum(dif.^2, 2);           % Cálculo rápido de cercanía sin aplicar raíces.
        [~, idx] = min(distancias_sq);            % Búsqueda de la rama más cercana al nuevo punto.
        nearest_node = current_nodes(idx, :);     % Obtención de las coordenadas exactas de dicha rama.

        direction = (rand_point - nearest_node);  % Trazado de un vector guía.
        direction = direction / norm(direction);  % Normalización del vector para movimiento puro.

        paso_actual = step_size;                  % Asignación de la longitud de crecimiento estándar.
        nodo_valido = false;                      % Bandera de validación del siguiente paso.

        while paso_actual >= 3                    % Ciclo de retracción dinámica para esquivar colisiones.
          new_node = nearest_node + paso_actual * direction; % Proyección de la posible nueva rama.

          if ~Verificar_Colision_Segmento(nearest_node, new_node, mapa_discreto) % Chequeo contra el mapa de edificios.
            nodo_valido = true;                   % Confirmación de camino despejado.
            break;                                % Salida del ciclo de retracción.
          else
            paso_actual = paso_actual / 2;        % Recorte de la rama a la mitad por impacto.
          end
        end

        if nodo_valido                                  % Acción condicional a la viabilidad del camino.
            current_nodes = [current_nodes; new_node];  % Integración de la rama al árbol.
            current_parent(end+1) = idx;                % Vinculación con la rama padre correspondiente.
            new_node_idx = size(current_nodes, 1);      % Almacenamiento del ID de la nueva rama.

            bosque_nodos{tree_id} = current_nodes;      % Actualización de la memoria del bosque.
            bosque_padres{tree_id} = current_parent;    % Actualización de genealogía.

            if tree_id == 1                       % Determinación de color azul para el inicio.
                color_rama = 'b';
            elseif tree_id == 2                   % Determinación de color cyan para el objetivo.
                color_rama = 'c';
            else                                  % Asignación de color verde para semillas secundarias.
                color_rama = 'g';
            end
            plot([nearest_node(1), new_node(1)], [nearest_node(2), new_node(2)], color_rama); % Trazado gráfico del crecimiento.

            if mod(i, 300) == 0                   % Regulación del refresco de pantalla para evitar lentitud.
                pause(0.001);                     % Pausa milimétrica.
            end

            for otro_id = 1:num_arboles           % Búsqueda de conexiones con otros elementos del bosque.
                if otro_id == tree_id || matriz_conexiones(tree_id, otro_id) == 1 % Exclusión de análisis redundantes.
                    continue;
                end

                other_nodes = bosque_nodos{otro_id};                    % Extracción de datos del árbol vecino.
                dif_other = other_nodes - new_node;                     % Medición de distancia interfamiliar.
                distancias_other_sq = sum(dif_other.^2, 2);             % Cálculo del grado de proximidad.
                [min_dist_sq, idx_nearest_other] = min(distancias_other_sq);  % Identificación del punto de choque potencial.

                if min_dist_sq < (tolerancia^2)                         % Filtro de proximidad según el radio permitido.
                    nodo_candidato = other_nodes(idx_nearest_other, :); % Selección del nodo blanco.

                    if ~Verificar_Colision_Segmento(new_node, nodo_candidato, mapa_discreto) % Verificación visual (línea de visión) entre ambos puntos.

                        matriz_conexiones(tree_id, otro_id) = 1;        % Registro oficial del puente bidireccional.
                        matriz_conexiones(otro_id, tree_id) = 1;

                        detalles_conexiones{tree_id, otro_id} = [new_node_idx, idx_nearest_other]; % Documentación técnica de los nodos involucrados.
                        detalles_conexiones{otro_id, tree_id} = [idx_nearest_other, new_node_idx];

                        % Representación visual de la unión en magenta.
                        plot([new_node(1), other_nodes(idx_nearest_other, 1)], [new_node(2), other_nodes(idx_nearest_other, 2)], 'm', 'LineWidth', 1);

                        if mod(i, 300) == 0 % Mantenimiento de la estabilidad gráfica.
                            pause(0.001);
                        end
                        disp(['> Conexión formada entre el árbol ', num2str(tree_id), ' y el árbol ', num2str(otro_id)]); % Aviso textual en consola.

                        if sum(matriz_conexiones(1,:)) > 0 && sum(matriz_conexiones(2,:)) > 0   % Revisión global para certificar unión Inicio-Fin.
                            ruta_temp = BFS_Encontrar_Ruta(matriz_conexiones, 1, 2);            % Invocación del rastreo algorítmico del camino.

                            if ~isempty(ruta_temp)                                  % Confirmación de viabilidad de la ruta encontrada.
                                % Extracción de la secuencia de coordenadas reales.
                                [~, ~, coords_temp, ~] = Unificar_Bosque_RRT(bosque_nodos, bosque_padres, detalles_conexiones, num_arboles);

                                costo_actual = 0;                                   % Preparación del medidor de distancia lineal.
                                for c = 1:(size(coords_temp, 1) - 1)                % Sumatoria geométrica segmento por segmento.
                                    costo_actual = costo_actual + norm(coords_temp(c+1, :) - coords_temp(c, :));
                                end

                                if costo_actual < mejor_costo                       % Evaluación de competitividad frente al récord actual.
                                    mejor_costo = costo_actual;                     % Sobrescritura del récord con el nuevo valor óptimo.
                                    path_coords_inicial = coords_temp;              % Respaldo de la ruta ganadora en memoria.

                                    if ~isempty(h_mejor_ruta)                       % Limpieza del dibujo obsoleto previo.
                                        delete(h_mejor_ruta);
                                    end

                                    % Despliegue gráfico de la ruta dominante en rojo.
                                    h_mejor_ruta = plot(path_coords_inicial(:,1), path_coords_inicial(:,2), 'r-', 'LineWidth', 2);

                                    if ~is_connected                                % Clasificación del hallazgo (primer éxito o mejora).
                                        disp(['¡Primera ruta encontrada! Costo: ', num2str(mejor_costo)]);
                                        is_connected = true;                        % Activación definitiva del estado de conexión global.
                                    else
                                        disp(['¡Atajo encontrado! Nuevo costo: ', num2str(mejor_costo)]);
                                    end
                                    drawnow;                                        % Actualización prioritaria del visor.
                                end
                            end
                        end
                    end
                end
            end % Fin del análisis de uniones vecinales.

        else
            nodos_colisionados = nodos_colisionados + 1;                            % Acumulación de estadísticas negativas por choques.
        end

        if is_connected                                               % Ejecución del límite de gracia RRT*.
            iteraciones_extra = iteraciones_extra + 1;                % Conteo regresivo hacia el cierre algorítmico.
            if iteraciones_extra > 370                                % Condición límite para abandonar la búsqueda de atajos.
                disp('¡Búsqueda RRT* optimizada finalizada!');
                break;                                                % Ruptura del ciclo infinito.
            end
        end
    end

    tiempo_busqueda_inicial = toc;                                    % Detención y registro del tiempo cronometrado.

    if is_connected                                                   % Protocolo de cierre en caso de solución satisfactoria.
        disp('Extrayendo la mejor ruta...');                          % Aviso de inicio de fase de post-procesamiento.

        costo_inicial = mejor_costo;                                  % Aislamiento del puntaje para su reporte.

        total_nodos_encontrados = 0;                                  % Recopilación de parámetros de esfuerzo.
        for k = 1:num_arboles                                         % Sumatoria del crecimiento bruto del bosque.
            total_nodos_encontrados = total_nodos_encontrados + size(bosque_nodos{k}, 1);
        end

        disp('Iniciando optimización de caminos (Suavizado por atajos)...'); % Información visual de refinamiento.

        if ~isempty(h_mejor_ruta)                                     % Purga de elementos visuales superpuestos.
            delete(h_mejor_ruta);
        end
        Limpiar_Dibujos_RRT('union');                                 % Eliminación de conexiones puente para evitar saturación visual.

        tic; % Arranque de medición exclusiva para el post-procesamiento.
        % Reducción de vértices y alineación de segmentos rectos.
        path_final_coords_completo = Optimizar_Ruta_Final(path_coords_inicial, mapa_discreto, @Verificar_Colision_Segmento);
        tiempo_optimizacion = toc;                                    % Registro de velocidad de limpieza.

        Limpiar_Dibujos_RRT('fase_optimizacion');                     % Supresión final de basura gráfica.

        % Consolidación del trazo maestro final.
        plot(path_final_coords_completo(:,1), path_final_coords_completo(:,2), 'r-', 'LineWidth', 2.5);
        % Remarcado estético de esquinas en negro y rojo.
        plot(path_final_coords_completo(:,1), path_final_coords_completo(:,2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'black');
        drawnow;                                                      % Confirmación visual definitiva.

        nodos_camino_final = size(path_final_coords_completo, 1);     % Métrica del nivel de simplificación geométrica.
        disp('Optimización finalizada. Camino más corto trazado.');   % Mensaje de cumplimiento total.

        % Envío de parámetros al generador de reportes .txt.
        Exportar_Reporte_Metricas(path_final_coords_completo, tiempo_busqueda_inicial, tiempo_optimizacion, total_nodos_encontrados, nodos_camino_final, iteraciones_totales, nodos_colisionados, costo_inicial);
    else
        disp('No se encontró conexión dentro del número máximo de iteraciones.'); % Mensaje por fallo crítico de conexión.
    end
end
