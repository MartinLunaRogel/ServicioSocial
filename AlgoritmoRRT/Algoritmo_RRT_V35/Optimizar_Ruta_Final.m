% =========================================================================
% === FUNCION DE POST-PROCESAMIENTO: OPTIMIZACIÓN DE RUTA FINAL ===
%
% Propósito: Esta función se encarga de "suavizar" o "alisar" el camino encontrado. Debido a que la exploración genera rutas con muchos vértices
% o movimientos en zigzag, este proceso intenta trazar líneas rectas entre puntos alejados de la ruta. Si la línea recta no cruza ningún obstáculo,
% se eliminan los puntos intermedios, creando un atajo directo.
% =========================================================================
function path_final_coords = Optimizar_Ruta_Final(path_coords, mapa_discreto, Verificar_Colision_Segmento)

    path_optimizado = path_coords;                        % Se hace una copia de la ruta original (con todos sus vértices) para comenzar a recortarla.
    max_intentos = 400;                                   % Se define un límite estricto de intentos de optimización (400) para evitar ciclos infinitos y ahorrar memoria.

    % --- CICLO DE CREACIÓN DE ATAJOS ---

    for k = 1:max_intentos                                % Se inicia un ciclo que se repetirá exactamente la cantidad de veces definida por el límite.

        num_nodos = size(path_optimizado, 1);             % Se cuenta la cantidad de nodos que tiene la ruta en este momento del ciclo.
        if num_nodos <= 2                                 % Se evalúa si la ruta ya quedó reducida a solo 2 puntos (el Inicio y el Objetivo absolutos).
            break;                                        % De ser así, se rompe el ciclo porque ya es una línea recta perfecta y no se puede optimizar más.
        end

        % --- SELECCIÓN ALEATORIA DE PUNTOS ---

        idx1 = randi(num_nodos - 2);                      % Se elige al azar un índice de la ruta para que sea el punto de partida del atajo.
                                                          % Se restan 2 posiciones al límite para dejar espacio suficiente para el segundo punto.

        idx2 = randi([idx1 + 2, num_nodos]);              % Se elige al azar un segundo índice más adelantado en la ruta para que sea el destino del atajo.
                                                          % Se asegura que este segundo punto esté al menos 2 posiciones por delante del primero.

        nodo_A = path_optimizado(idx1, :);                % Se extraen las coordenadas (X, Y) reales que corresponden al primer índice elegido.
        nodo_B = path_optimizado(idx2, :);                % Se extraen las coordenadas (X, Y) reales que corresponden al segundo índice elegido.

        % --- VERIFICACIÓN Y CORTE (SHORTCUTTING) ---

        % Se utiliza el motor de colisiones de Bresenham para trazar un "láser" imaginario entre ambos puntos.
        if ~Verificar_Colision_Segmento(nodo_A, nodo_B, mapa_discreto)          % Si no hay colision

            path_optimizado(idx1+1 : idx2-1, :) = [];     % Si la condición se cumple (el atajo es un espacio vacío), se procede a borrar la basura.
                                                          % Se eliminan de la lista todos los puntos que existían entre el índice 1 y el índice 2.
        end
    end

    path_final_coords = path_optimizado;                  % Al finalizar todos los intentos, se devuelve la ruta depurada y recortada al programa principal.
end
