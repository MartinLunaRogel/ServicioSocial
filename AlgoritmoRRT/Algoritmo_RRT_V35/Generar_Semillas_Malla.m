% =========================================================================
% === FUNCION PARA GENERAR SEMILLAS UNIFORMES SOBRE EL EJE MEDIAL ===
% =========================================================================

function semillas = Generar_Semillas_Malla(coordenadas_esqueleto, separacion)
    semillas = [];

    if isempty(coordenadas_esqueleto)
        return;
    end

    % 1. Mezclar las coordenadas al azar para no sesgar el acomodo
    idx_mezclados = randperm(size(coordenadas_esqueleto, 1));
    coords_random = coordenadas_esqueleto(idx_mezclados, :);

    % 2. La primera coordenada del esqueleto siempre se acepta como semilla
    semillas = coords_random(1, :);

    % 3. Iterar sobre el resto del esqueleto
    for i = 2:size(coords_random, 1)
        punto_candidato = coords_random(i, :);

        % Calcular la distancia desde este candidato a TODAS las semillas ya plantadas
        distancias = sqrt((semillas(:,1) - punto_candidato(1)).^2 + (semillas(:,2) - punto_candidato(2)).^2);

        % Si el candidato está lo suficientemente lejos de todas las demás semillas, lo plantamos
        if min(distancias) >= separacion
            semillas = [semillas; punto_candidato];
        end
    end
end
