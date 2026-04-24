% main.m
% =========================================================================
% === SISTEMA DE PLANIFICACIÓN DE RUTAS RRT* (Versión Base Funcional) ===
% =========================================================================
clc; clear; close all;

% LÍNEA DE SEGURIDAD: Fuerza a Octave a trabajar en la carpeta de este archivo
% Esto elimina el error "[Errno 2] No such file or directory"
cd(fileparts(mfilename('fullpath')));

disp('Iniciando Sistema RRT*...');

opcion = 0;
while opcion ~= 3
    opcion = menu('Menú Principal - RRT*', ...
                  '1. Analizar archivo Mapa .osm', ...
                  '2. Ejecutar Algoritmo RRT*', ...
                  '3. Salir');

    if opcion == 1
        disp('--- INICIANDO ANÁLISIS DE MAPA OSM ---');

        [nombre_archivo, ruta_archivo] = uigetfile('*.osm', 'Selecciona el mapa exportado de OpenStreetMap');

        if isequal(nombre_archivo, 0)
            disp('Operación cancelada.');
        else
            ruta_completa = fullfile(ruta_archivo, nombre_archivo);

            % Llamada directa a Python
            comando_python = sprintf('py analizar_osm.py "%s"', ruta_completa);

            disp('Ejecutando motor de análisis en Python...');
            [estado, resultado] = system(comando_python);
            disp(resultado);

            if estado == 0
                disp('Análisis completado. Listo para simulación.');
            else
                disp('Ocurrió un error en el motor de Python.');
            end
        end
        disp('--------------------------------------');

    elseif opcion == 2
        disp('--- INICIANDO SIMULACIÓN RRT* ---');
        run('Ejecutar_RRT_Star.m');
        disp('--------------------------------------');

    elseif opcion == 3
        disp('Cerrando sistema. ¡Hasta luego!');
    end
end
