% Obstaculos_Para_RRT.m (ENTORNO LABERINTO CON FIGURAS VARIADAS)
% Define obstáculos poligonales con diversas formas (Triángulo, L, Pentágono, C, Trapecio)
% que obligan al RRT a tomar rutas muy indirectas.

% Cada elemento de 'obstaculos' es una matriz Nx2 [X_coords, Y_coords]
obstaculos = {
    % =========================================================================
    % 1. Bloqueo Central (Obliga al movimiento a través de los bordes)
    % =========================================================================

    % Obstáculo 1: Gran pared vertical (Rectángulo)
    % Bloquea ruta directa central. Deja pasajes de 10 unidades en Y=0-10 y Y=90-100.
    [45, 10;
     55, 10;
     55, 90;
     45, 90];

    % Obstáculo 2: Pared horizontal baja (Rectángulo)
    % Obliga al punto de inicio [10, 20] a moverse hacia arriba.
    [15, 30;
     45, 30;
     45, 33;
     15, 33];

    % =========================================================================
    % 2. Variedad de Formas Geométricas (Dificultad de Navegación)
    % =========================================================================

    % Obstáculo 3: Forma de L Irregular (6 vértices)
    % Crea una esquina de difícil paso en el cuadrante superior izquierdo.
    [20, 55;
     35, 55;
     35, 75;
     30, 75;
     30, 60;
     20, 60];

    % Obstáculo 4: Pentágono Irregular (5 vértices)
    % Forma compleja en el centro-derecha.
    [65, 40;
     80, 45;
     85, 60;
     75, 70;
     60, 60];

    % Obstáculo 5: Forma de C o Herradura (8 vértices)
    % Trampa o callejón sin salida en el cuadrante inferior derecho.
    [60, 15;
     90, 15;
     90, 35;
     85, 35;
     85, 20;
     65, 20;
     65, 30;
     60, 30];

    % Obstáculo 6: Trapecio (4 vértices con lados no paralelos al eje)
    % Bloquea una ruta diagonal hacia el objetivo [90, 80].
    [58, 70;
     80, 80;
     75, 85;
     53, 75];

    % Obstáculo 7: Triángulo (3 vértices)
    % Pequeño bloqueo cerca del borde superior.
    [10, 80;
     20, 95;
     30, 80];
};
