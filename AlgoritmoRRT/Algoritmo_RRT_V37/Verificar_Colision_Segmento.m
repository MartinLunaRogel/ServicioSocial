% =========================================================================
% === FUNCION DE COLISIONES: RAY-CASTING DISCRETO (BRESENHAM) ===
%
% Esta función actúa como un sensor de choques del árbol.
% Cuando el árbol quiere crecer del Nodo A al Nodo B, esta función traza
% una línea recta (píxel por píxel) sobre el mapa.
% Si esa línea toca algún obstáculo (los numeros "1" en la matriz discreta) significa que el camino está bloqueado.
% =========================================================================
function hay_colision = Verificar_Colision_Segmento(node1, node2, mapa_discreto)

    % --- CONVERSIÓN DE COORDENADAS (rEALES A PÍXELES) ---

    % Las coordenadas que llegan a la funcion tienen decimales, pero la matriz del mapa es de enteros.
    % Se redondea el valor de x del primer punto
    x1 = round(node1(1)) + 1;
    y1 = round(node1(2)) + 1; % Se hace lo mismo para el eje Y del primer punto
    x2 = round(node2(1)) + 1; % Redondeamos X del segundo punto
    y2 = round(node2(2)) + 1; % Redondeamos Y del segundo punto

    % --- PROTECCIÓN DE LÍMITES (EVITAR QUE EL LÁSER SALGA DE LA PANTALLA) ---

    % Se el pregunta a la matriz del mapa discreto de cuanto es su tamaño.
    [max_filas, max_cols] = size(mapa_discreto);

    % Si el punto cae fuera del area se mueve para que quede dentro del mapa
    x1 = max(1, min(x1, max_cols));
    y1 = max(1, min(y1, max_filas));
    x2 = max(1, min(x2, max_cols));
    y2 = max(1, min(y2, max_filas));

    % --- CONFIGURACIÓN DEL ESCÁNER (ALGORITMO DE BRESENHAM) ---

    % Calculamos la distancia total que el paso recorrerá en el eje horizontal (X).
    dx = abs(x2 - x1);

    % Calculamos la distancia total que el paso recorrerá en el eje vertical (Y).
    dy = abs(y2 - y1);

    % Averiguamos en qué dirección nos movemos horizontalmente (1 = derecha, -1 = izquierda).
    sx = sign(x2 - x1);

    % Averiguamos en qué dirección nos movemos verticalmente (1 = arriba, -1 = abajo).
    sy = sign(y2 - y1);

    % Calculamos el margen de error
    % el siguiente píxel en diagonal en lugar de ir en línea recta.
    err = dx - dy;

    % Colocamos nuestro escáner en el punto de inicio.
    x = x1;
    y = y1;

    % --- ESCANEO PÍXEL POR PÍXEL DE LA LÍNEA ---

    % Iniciamos un ciclo. Iremos dando pasos en la cuadrícula hasta llegar al destino o chocar.
    while true

        %  Se recisa la celda actual en el mapa.
        if mapa_discreto(y, x) == 1

            % Encontramos un 1 (significa pared). El paso no puede pasar.
            hay_colision = true;

            % Se aborta la funcion.
            return;
        end

        % Revisamos si nuestro picel del scanner ya llegó a la coordenada final exacta.
        if x == x2 && y == y2

            % Si ya llegó al final y no ha chocado con nada, rompemos el ciclo.
            break;
        end

        % Preparamos el margen de error para el siguiente paso.
        e2 = 2 * err;

        % Si toca moverse horizontalmente, actualizamos el error y damos un paso en X.
        if e2 > -dy
            err = err - dy;
            x = x + sx;
        end

        % Si toca moverse verticalmente, actualizamos el error y damos un paso en Y.
        if e2 < dx
            err = err + dx;
            y = y + sy;
        end
    end

    % --- RESULTADO FINAL (CAMINO LIBRE) ---

    % Si logramos salir del ciclo (con el comando 'break'),
    % significa que recorrimos toda la línea sin tocar ningún '1'. ¡Es un paso seguro!
    hay_colision = false;
end
