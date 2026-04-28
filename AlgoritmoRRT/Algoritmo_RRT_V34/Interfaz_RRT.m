function Interfaz_RRT()

    % Forzar a Octave a ubicarse en la carpeta correcta
    cd(fileparts(mfilename('fullpath')));

    % Limpiar variables globales por si ejecutaste el programa antes
    clear global punto_inicio punto_final obstaculos;
    global obstaculos punto_inicio punto_final mapa_listo;
    mapa_listo = false;

    % 1. Crear la ventana principal
    fig = figure('Name', 'Navegador RRT*, ...
                 'Position', [100 100 800 600], ...
                 'MenuBar', 'none', ...
                 'NumberTitle', 'off');

    % 2. Crear el área del mapa (axes) con UNIDADES NORMALIZADAS
    % Posición: [X Y Ancho Alto] en porcentajes (0.0 a 1.0)
    ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.25 0.05 0.70 0.90]);
    title(ax, 'Mapa de Obstáculos');
    xlabel(ax, 'X'); ylabel(ax, 'Y');
    set(ax, 'XLim', [0 500], 'YLim', [0 500]);
    axis(ax, 'equal'); % ¡SUPER IMPORTANTE! Evita que el mapa se deforme al maximizar
    grid(ax, 'on'); hold(ax, 'on');

    % 3. Crear el panel lateral de botones con UNIDADES NORMALIZADAS
    uicontrol('Parent', fig, 'Style', 'text', 'String', 'CONTROLES', ...
              'Units', 'normalized', 'Position', [0.02 0.85 0.20 0.05], ...
              'FontSize', 12, 'FontWeight', 'bold', 'BackgroundColor', get(fig, 'Color'));

    % Botón 1: Analizar Mapa
    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '1. Analizar Mapa', ...
              'Units', 'normalized', 'Position', [0.02 0.75 0.20 0.08], 'Callback', @analizarMapa);

    % Botón 2: Punto Inicio
    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '2. Punto Inicio', ...
              'Units', 'normalized', 'Position', [0.02 0.65 0.20 0.08], 'Callback', @seleccionarInicio);

    % Botón 3: Punto Fin
    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '3. Punto Fin', ...
              'Units', 'normalized', 'Position', [0.02 0.55 0.20 0.08], 'Callback', @seleccionarFin);

    % Botón 4: Iniciar Búsqueda
    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '4. Iniciar Búsqueda', ...
              'Units', 'normalized', 'Position', [0.02 0.45 0.20 0.08], ...
              'BackgroundColor', [0.8 1 0.8], 'Callback', @ejecutarRRT);

    % --- FUNCIONES DE LOS BOTONES ---

    function analizarMapa(~, ~)
        [archivo, ruta] = uigetfile('*.osm', 'Selecciona el mapa OSM');
        if archivo ~= 0
            comando = sprintf('py analizar_osm.py "%s"', fullfile(ruta, archivo));
            [status, out] = system(comando);
            disp(out);
            if status == 0
                evalin('base', 'Mapa_Obstaculos_RRT');
                obstaculos = evalin('base', 'obstaculos');
                mapa_listo = true;

                % Romper la cadena hacia adelante: Borramos puntos anteriores
                punto_inicio = [];
                punto_final = [];

                % Limpiar y dibujar solo el mapa
                cla(ax); hold(ax, 'on');
                for k = 1:size(obstaculos, 1)
                    patch(obstaculos{k,1}(:,1), obstaculos{k,1}(:,2), [0.3 0.3 0.3], 'Parent', ax);
                end
                set(ax, 'XLim', [0 500], 'YLim', [0 500]);
                axis(ax, 'equal'); grid(ax, 'on');
                msgbox('Mapa cargado y procesado.', 'Éxito');
            end
        end
    end

    function seleccionarInicio(~, ~)
        if ~mapa_listo
            errordlg('Carga el mapa primero.', 'Error'); return;
        end

        % Romper la cadena: Invalidamos el punto final
        punto_final = [];

        % 1. Limpiar todo visualmente (árboles, puntos viejos, metas)
        cla(ax); hold(ax, 'on');

        % 2. Redibujar solo los obstáculos
        for k = 1:size(obstaculos, 1)
            patch(obstaculos{k,1}(:,1), obstaculos{k,1}(:,2), [0.3 0.3 0.3], 'Parent', ax);
        end
        set(ax, 'XLim', [0 500], 'YLim', [0 500]);
        axis(ax, 'equal'); grid(ax, 'on');

        % 3. Pedir el nuevo Inicio
        title(ax, 'Haz clic en el mapa para el INICIO');
        axes(ax);
        [ix, iy] = ginput(1);

        % =========================================================
        % --- NUEVO: VALIDACIÓN DE COLISIÓN DEL MOUSE ---
        dentro_de_obstaculo = false;
        for k = 1:size(obstaculos, 1)
            poligono = obstaculos{k, 1};
            % inpolygon revisa si el clic cayó dentro del bloque gris
            if inpolygon(ix, iy, poligono(:,1), poligono(:,2))
                dentro_de_obstaculo = true;
                break;
            end
        end

        if dentro_de_obstaculo
            errordlg('El punto está dentro de un area indevida. Por favor, selecciona una calle libre.', 'Punto Inválido');
            title(ax, 'Error: Selecciona el inicio en una calle.');
            return; % Cancelamos la acción y no guardamos el punto
        end
        % =========================================================

        % 4. Registrar y dibujar el nuevo Inicio válido
        punto_inicio = [ix, iy];
        plot(ax, ix, iy, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
        title(ax, 'Inicio fijado. Selecciona el Objetivo.');
    end

    function seleccionarFin(~, ~)
        if ~mapa_listo
            errordlg('Carga el mapa primero.', 'Error'); return;
        end

        % Candado de dependencia
        if isempty(punto_inicio)
            errordlg('Selecciona el punto de INICIO primero.', 'Error'); return;
        end

        % 1. Limpiar visualmente (Borra árboles y el objetivo viejo)
        cla(ax); hold(ax, 'on');

        % 2. Redibujar los obstáculos
        for k = 1:size(obstaculos, 1)
            patch(obstaculos{k,1}(:,1), obstaculos{k,1}(:,2), [0.3 0.3 0.3], 'Parent', ax);
        end

        % 3. Redibujar el Inicio (Porque ese eslabón no se rompió)
        plot(ax, punto_inicio(1), punto_inicio(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

        set(ax, 'XLim', [0 500], 'YLim', [0 500]);
        axis(ax, 'equal'); grid(ax, 'on');

        % 4. Pedir el nuevo Objetivo
        title(ax, 'Haz clic en el mapa para el OBJETIVO');
        axes(ax);
        [ox, oy] = ginput(1);

        % =========================================================
        % --- NUEVO: VALIDACIÓN DE COLISIÓN DEL MOUSE ---
        dentro_de_obstaculo = false;
        for k = 1:size(obstaculos, 1)
            poligono = obstaculos{k, 1};
            if inpolygon(ox, oy, poligono(:,1), poligono(:,2))
                dentro_de_obstaculo = true;
                break;
            end
        end

        if dentro_de_obstaculo
            errordlg('El objetivo está dentro de un area indevida. Por favor, selecciona una calle libre.', 'Punto Inválido');
            title(ax, 'Error: Selecciona la meta en una calle.');
            return; % Cancelamos la acción y no guardamos el punto
        end
        % =========================================================

        % 5. Registrar y dibujar el nuevo Objetivo válido
        punto_final = [ox, oy];
        plot(ax, ox, oy, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        title(ax, 'Objetivo fijado. Listo para buscar.');
    end

    function ejecutarRRT(~, ~)
        if isempty(punto_inicio) || isempty(punto_final)
            errordlg('Selecciona inicio y fin primero.', 'Error'); return;
        end

        % =====================================================================
        % === NUEVO: REFRESCAR EL MAPA ANTES DE EMPEZAR ===
        % =====================================================================
        % Borramos todos los árboles y rutas de la búsqueda anterior
        cla(ax);
        hold(ax, 'on');

        % 1. Redibujamos los obstáculos grises (están en la variable global)
        for k = 1:size(obstaculos, 1)
            patch(obstaculos{k,1}(:,1), obstaculos{k,1}(:,2), [0.3 0.3 0.3], 'Parent', ax);
        end

        % 2. Redibujamos los puntos Verde (Inicio) y Rojo (Meta)
        plot(ax, punto_inicio(1), punto_inicio(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
        plot(ax, punto_final(1), punto_final(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

        % 3. Restauramos la configuración visual del mapa
        set(ax, 'XLim', [0 500], 'YLim', [0 500]);
        axis(ax, 'equal');
        grid(ax, 'on');
        % =====================================================================

        title(ax, 'Calculando nueva ruta...');
        axes(ax); % Asegurar que el algoritmo dibuje en el cuadro correcto

        % Sincronizar variables con la memoria base de Octave
        assignin('base', 'punto_inicio', punto_inicio);
        assignin('base', 'punto_final', punto_final);
        assignin('base', 'obstaculos', obstaculos);

        evalin('base', 'global punto_inicio punto_final obstaculos;');
        evalin('base', 'Ejecutar_RRT_Star');

        title(ax, '¡Búsqueda Finalizada!');
    end
end
