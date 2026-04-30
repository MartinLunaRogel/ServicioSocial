% =========================================================================
% === FUNCION DE DISTRIBUCIÓN: GENERACIÓN DE SEMILLAS EN LA MALLA ===
%
% Aactúa como un jardin. Toma el esqueleto de las calles (la línea central de todas las calles de la ciudad) y planta
% "semillas" (puntos de inicio para nuevos árboles) a lo largo de ellas.
% Para no saturar el mapa, se asegura de que cada semilla esté separada de las demás por una distancia mínima (separacion).
% =========================================================================
function semillas = Generar_Semillas_Malla(coordenadas_esqueleto, separacion)

    % Se prepara una lista donde iremos guardando las semillas
    semillas = [];

    % Si el mapa no tiene calles (la lista de coordenadas está vacía)
    if isempty(coordenadas_esqueleto)
        % se aborta y re retorna la lista vacia
        return;
    end

    % --- Se "Aleétorizan" las semillas ---

    % randperm genera una lista de números revueltos desde el 1 hasta el total de coordenadas que tenemos.
    idx_mezclados = randperm(size(coordenadas_esqueleto, 1));

    % Se aplica ese orden a la lista original de coordenadas de las calles.
    % Ahora se tienen la misma lista de calles, pero en un orden aleatorio.
    coords_random = coordenadas_esqueleto(idx_mezclados, :);

    % --- GENERAR LA PRIMERA SEMILLA ---

    % Como LA LISTA está vacía, no hay riesgo de chocar con otra semilla.
    % Así que tomamos el primer punto de nuestra lista revuelta y lo plantamos directamente.
    semillas = coords_random(1, :);

    % --- CICLO DE GENERACION (REVISAR Y PLANTAR EL RESTO) ---

    % Empezamos un ciclo desde el segundo punto de la lista hasta el último
    for i = 2:size(coords_random, 1)

        % Tomamos el punto que nos toca revisar en este turno y lo llamamos "candidato"
        punto_candidato = coords_random(i, :);

        % --- APLICAR TEOREMA DE PITÁGORAS ---
        % Calculamos la distancia en línea recta desde nuestro candidato hacia TODAS las semillas
        % Fórmula: Raíz cuadrada de ((X2 - X1)^2 + (Y2 - Y1)^2)
        distancias = sqrt((semillas(:,1) - punto_candidato(1)).^2 + (semillas(:,2) - punto_candidato(2)).^2);

        % Revisamos cuál de todas esas distancias es la más pequeña (es decir, cuál es la semilla más cercana).
        % Si la semilla más cercana está a una distancia IGUAL O MAYOR a nuestra 'separacion' requerida (DE 50)...
        if min(distancias) >= separacion

            % ...¡Significa que hay espacio libre!
            % Agregamos este candidato a nuestra lista de semillas plantadas.
            semillas = [semillas; punto_candidato];
        end

        % Si la distancia era menor, el 'if' se ignora (el punto se descarta porque está muy cerca)
        % y pasamos al siguiente candidato del ciclo 'for'.
    end
end
