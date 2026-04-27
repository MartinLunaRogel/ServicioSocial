% main.m - Versión 26 Interactiva (Corrección ginput)
clc; clear; close all;
cd(fileparts(mfilename('fullpath')));

% Variables globales para persistencia
global obstaculos punto_inicio punto_final mapa_listo puntos_listos;
mapa_listo = false; puntos_listos = false;

while true
    opcion = menu('SISTEMA RRT* QRO', ...
                  '1. Generar/Cargar Mapa OSM', ...
                  '2. Seleccionar Inicio y Objetivo (Mouse)', ...
                  '3. Ejecutar Algoritmo RRT*', ...
                  '4. Salir');

    switch opcion
        case 1
            [archivo, ruta] = uigetfile('*.osm', 'Selecciona el mapa OSM');
            if archivo ~= 0
                comando = sprintf('py analizar_osm.py "%s"', fullfile(ruta, archivo));
                [status, out] = system(comando);
                disp(out);
                if status == 0
                    run('Mapa_Obstaculos_RRT.m');
                    mapa_listo = true;
                    msgbox('Mapa cargado.', 'Éxito');
                end
            end

        case 2
            if ~mapa_listo
                errordlg('Primero carga el mapa (Opción 1).');
            else
                fig = figure('Name', 'Seleccionador de Puntos');
                hold on; grid on; axis([0 500 0 500]);

                % Dibujar los obstáculos generados
                for k = 1:size(obstaculos, 1)
                    patch(obstaculos{k,1}(:,1), obstaculos{k,1}(:,2), [0.3 0.3 0.3]);
                end

                title('1. Haz clic para el INICIO (Verde)');
                % CORRECCIÓN: Extraer X y Y explícitamente
                [ix, iy] = ginput(1);
                punto_inicio = [ix, iy];
                plot(ix, iy, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

                title('2. Haz clic para el OBJETIVO (Rojo)');
                % CORRECCIÓN: Extraer X y Y explícitamente
                [ox, oy] = ginput(1);
                punto_final = [ox, oy];
                plot(ox, oy, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

                puntos_listos = true;
                title('Puntos guardados. Cerrando...');
                pause(1); close(fig);
            end

        case 3
            if mapa_listo && puntos_listos
                run('Ejecutar_RRT_Star.m');
            else
                errordlg('Faltan datos. Carga mapa y puntos primero.');
            end

        case 4
            break;
    end
end
