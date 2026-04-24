% Generar_Semillas_Quadtree.m
% =========================================================================
% === FUNCION DE ANALISIS GEOMETRICO PARA DETECTAR CALLES (QUADTREE) ===
% =========================================================================

function semillas = Generar_Semillas_Quadtree(x_min, x_max, y_min, y_max, obstaculos, min_size)
    semillas = [];

    % 1. Calcular dimensiones y centro del bloque actual
    width = x_max - x_min;
    height = y_max - y_min;
    centro = [x_min + width/2, y_min + height/2];

    % 2. Probar 5 puntos clave para ver si el bloque toca un obstáculo
    % (Esquinas + Centro)
    puntos_test = [x_min, y_min; x_max, y_min; x_min, y_max; x_max, y_max; centro];
    colisiones = 0;

    for p = 1:size(puntos_test, 1)
        px = puntos_test(p,1);
        py = puntos_test(p,2);

        for k = 1:size(obstaculos, 1)
            if inpolygon(px, py, obstaculos{k,1}(:,1), obstaculos{k,1}(:,2))
                colisiones = colisiones + 1;
                break; % Ya chocó con un edificio, pasamos al siguiente punto de test
            end
        end
    end

    % 3. Lógica de Decisión (Subdivisión)
    % Si el bloque es MIXTO (algunos puntos chocan y otros no) y aún es grande
    if colisiones > 0 && colisiones < 5 && width > min_size
        x_mid = x_min + width/2;
        y_mid = y_min + height/2;

        % Subdividimos en 4 cuadrantes y exploramos cada uno
        s1 = Generar_Semillas_Quadtree(x_min, x_mid, y_min, y_mid, obstaculos, min_size);
        s2 = Generar_Semillas_Quadtree(x_mid, x_max, y_min, y_mid, obstaculos, min_size);
        s3 = Generar_Semillas_Quadtree(x_min, x_mid, y_mid, y_max, obstaculos, min_size);
        s4 = Generar_Semillas_Quadtree(x_mid, x_max, y_mid, y_max, obstaculos, min_size);
        s4 = Generar_Semillas_Quadtree(x_mid, x_max, y_mid, y_max, obstaculos, min_size);

        semillas = [s1; s2; s3; s4];

    elseif colisiones == 0
        % Si el bloque está TOTALMENTE LIBRE de edificios, ponemos la semilla
        semillas = centro;
    end
end
