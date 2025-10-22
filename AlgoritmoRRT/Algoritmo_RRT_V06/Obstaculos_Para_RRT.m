% Obstaculos_ParaRRT.m
% Definición de un patrón de damero 8x8 para el algoritmo RRT.
% Bloques de 7x7 con 5 unidades de espacio libre entre ellos (Ciclo 12).
% Se excluyen las zonas alrededor de Inicio [10, 20] y Objetivo [90, 80].

% Puntos de Exclusión:
% La exclusión se basa en el ciclo de 12.
% Inicio [10, 20]: Excluye los bloques con X_min en [1] y [13] y Y_min en [1] y [13].
% Objetivo [90, 80]: Excluye los bloques con X_min en [85] y Y_min en [73].

obstaculos = {

    % === Fila 1 (Y_min = 1) | Y_max = 7 ===
    % Ciclo X_min: [1, 13, 25, 37, 49, 61, 73, 85, 97]
    % Inicio [10, 20] excluye X_min=1, 13

    % Bloque (1, 3): X en [25, 31], Y en [1, 7]
    [25, 31], [1, 7];

    % Bloque (1, 4): X en [37, 43], Y en [1, 7]
    [37, 43], [1, 7];

    % Bloque (1, 5): X en [49, 55], Y en [1, 7]
    [49, 55], [1, 7];

    % Bloque (1, 6): X en [61, 67], Y en [1, 7]
    [61, 67], [1, 7];

    % Bloque (1, 7): X en [73, 79], Y en [1, 7]
    [73, 79], [1, 7];

    % Bloque (1, 8): X en [85, 91], Y en [1, 7]
    [85, 91], [1, 7];


    % === Fila 2 (Y_min = 13) | Y_max = 19 ===
    % Inicio [10, 20] excluye X_min=1, 13

    % Bloque (2, 3): X en [25, 31], Y en [13, 19]
    [25, 31], [13, 19];

    % Bloque (2, 4): X en [37, 43], Y en [13, 19]
    [37, 43], [13, 19];

    % Bloque (2, 5): X en [49, 55], Y en [13, 19]
    [49, 55], [13, 19];

    % Bloque (2, 6): X en [61, 67], Y en [13, 19]
    [61, 67], [13, 19];

    % Bloque (2, 7): X en [73, 79], Y en [13, 19]
    [73, 79], [13, 19];

    % Bloque (2, 8): X en [85, 91], Y en [13, 19]
    [85, 91], [13, 19];


    % === Fila 3 (Y_min = 25) | Y_max = 31 ===
    % Ninguna exclusión en Y.

    % Bloque (3, 1): X en [1, 7], Y en [25, 31]
    [1, 7], [25, 31];

    % Bloque (3, 2): X en [13, 19], Y en [25, 31]
    [13, 19], [25, 31];

    % Bloque (3, 3): X en [25, 31], Y en [25, 31]
    [25, 31], [25, 31];

    % Bloque (3, 4): X en [37, 43], Y en [25, 31]
    [37, 43], [25, 31];

    % Bloque (3, 5): X en [49, 55], Y en [25, 31]
    [49, 55], [25, 31];

    % Bloque (3, 6): X en [61, 67], Y en [25, 31]
    [61, 67], [25, 31];

    % Bloque (3, 7): X en [73, 79], Y en [25, 31]
    [73, 79], [25, 31];

    % Bloque (3, 8): X en [85, 91], Y en [25, 31]
    [85, 91], [25, 31];


    % === Fila 4 (Y_min = 37) | Y_max = 43 ===

    % Bloque (4, 1): X en [1, 7], Y en [37, 43]
    [1, 7], [37, 43];

    % Bloque (4, 2): X en [13, 19], Y en [37, 43]
    [13, 19], [37, 43];

    % Bloque (4, 3): X en [25, 31], Y en [37, 43]
    [25, 31], [37, 43];

    % Bloque (4, 4): X en [37, 43], Y en [37, 43]
    [37, 43], [37, 43];

    % Bloque (4, 5): X en [49, 55], Y en [37, 43]
    [49, 55], [37, 43];

    % Bloque (4, 6): X en [61, 67], Y en [37, 43]
    [61, 67], [37, 43];

    % Bloque (4, 7): X en [73, 79], Y en [37, 43]
    [73, 79], [37, 43];

    % Bloque (4, 8): X en [85, 91], Y en [37, 43]
    [85, 91], [37, 43];


    % === Fila 5 (Y_min = 49) | Y_max = 55 ===

    % Bloque (5, 1): X en [1, 7], Y en [49, 55]
    [1, 7], [49, 55];

    % Bloque (5, 2): X en [13, 19], Y en [49, 55]
    [13, 19], [49, 55];

    % Bloque (5, 3): X en [25, 31], Y en [49, 55]
    [25, 31], [49, 55];

    % Bloque (5, 4): X en [37, 43], Y en [49, 55]
    [37, 43], [49, 55];

    % Bloque (5, 5): X en [49, 55], Y en [49, 55]
    [49, 55], [49, 55];

    % Bloque (5, 6): X en [61, 67], Y en [49, 55]
    [61, 67], [49, 55];

    % Bloque (5, 7): X en [73, 79], Y en [49, 55]
    [73, 79], [49, 55];

    % Bloque (5, 8): X en [85, 91], Y en [49, 55]
    [85, 91], [49, 55];


    % === Fila 6 (Y_min = 61) | Y_max = 67 ===

    % Bloque (6, 1): X en [1, 7], Y en [61, 67]
    [1, 7], [61, 67];

    % Bloque (6, 2): X en [13, 19], Y en [61, 67]
    [13, 19], [61, 67];

    % Bloque (6, 3): X en [25, 31], Y en [61, 67]
    [25, 31], [61, 67];

    % Bloque (6, 4): X en [37, 43], Y en [61, 67]
    [37, 43], [61, 67];

    % Bloque (6, 5): X en [49, 55], Y en [61, 67]
    [49, 55], [61, 67];

    % Bloque (6, 6): X en [61, 67], Y en [61, 67]
    [61, 67], [61, 67];

    % Bloque (6, 7): X en [73, 79], Y en [61, 67]
    [73, 79], [61, 67];

    % Bloque (6, 8): X en [85, 91], Y en [61, 67]
    [85, 91], [61, 67];


    % === Fila 7 (Y_min = 73) | Y_max = 79 ===
    % Objetivo [90, 80] excluye X_min=85 y Y_min=73

    % Bloque (7, 1): X en [1, 7], Y en [73, 79]
    [1, 7], [73, 79];

    % Bloque (7, 2): X en [13, 19], Y en [73, 79]
    [13, 19], [73, 79];

    % Bloque (7, 3): X en [25, 31], Y en [73, 79]
    [25, 31], [73, 79];

    % Bloque (7, 4): X en [37, 43], Y en [73, 79]
    [37, 43], [73, 79];

    % Bloque (7, 5): X en [49, 55], Y en [73, 79]
    [49, 55], [73, 79];

    % Bloque (7, 6): X en [61, 67], Y en [73, 79]
    [61, 67], [73, 79];

    % Bloque (7, 7): X en [73, 79], Y en [73, 79]
    [73, 79], [73, 79];


    % === Fila 8 (Y_min = 85) | Y_max = 91 ===
    % Objetivo [90, 80] excluye X_min=85

    % Bloque (8, 1): X en [1, 7], Y en [85, 91]
    [1, 7], [85, 91];

    % Bloque (8, 2): X en [13, 19], Y en [85, 91]
    [13, 19], [85, 91];

    % Bloque (8, 3): X en [25, 31], Y en [85, 91]
    [25, 31], [85, 91];

    % Bloque (8, 4): X en [37, 43], Y en [85, 91]
    [37, 43], [85, 91];

    % Bloque (8, 5): X en [49, 55], Y en [85, 91]
    [49, 55], [85, 91];

    % Bloque (8, 6): X en [61, 67], Y en [85, 91]
    [61, 67], [85, 91];

    % Bloque (8, 7): X en [73, 79], Y en [85, 91]
    [73, 79], [85, 91];
};
