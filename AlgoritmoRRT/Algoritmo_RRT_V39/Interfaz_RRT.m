% =========================================================================
% === CORE GRÁFICO: DASHBOARD ACADÉMICO DE NAVEGACIÓN AUTÓNOMA ===
%
% Interfaz de nivel de tesis desarrollada para entornos MATLAB/Octave.
% Incorpora selector de versiones algorítmicas para análisis comparativo
% histórico, preservando una arquitectura modular estricta.
% =========================================================================

function Interfaz_RRT()

    % Forzar el directorio activo a la ubicación de este script
    cd(fileparts(mfilename('fullpath')));

    % Agregar la carpeta de módulos históricos a la ruta de búsqueda
    addpath(fullfile(pwd, 'Versiones_Historicas'));

    try
        pkg load statistics; % Carga silenciosa del paquete para GNU Octave
    catch
        % Si se ejecuta en MATLAB o el paquete ya está cargado, no hace nada
    end

    % Depuración y liberación controlada de memoria caché global
    clear global punto_inicio punto_final obstaculos mapa_esqueleto coordenadas_esqueleto mapa_discreto semillas ax;

    % Declaración de variables globales de enlace inter-modular
    global obstaculos punto_inicio punto_final mapa_listo mapa_esqueleto coordenadas_esqueleto mapa_discreto semillas ax;

    mapa_actual_nombre = 'Sin_Asignar';
    mapa_listo = false;

    % ---------------------------------------------------------------------
    % PROPIEDADES ESTÉTICAS Y PALETA DE COLORES (CRITERIO FORMAL UAQ)
    % ---------------------------------------------------------------------
    c_uaq_azul      = [0.03, 0.13, 0.27];
    c_uaq_oro       = [0.76, 0.61, 0.18];
    c_bg_lienzo     = [0.96, 0.96, 0.97];
    c_panel_lateral = [0.06, 0.20, 0.38];
    c_text_light    = [0.95, 0.95, 0.95];

    % Paleta de Botones y Estados
    c_btn_base      = [0.22, 0.29, 0.36];
    c_btn_action    = [0.11, 0.41, 0.27];
    c_btn_reset     = [0.50, 0.10, 0.10];
    c_btn_run       = [0.11, 0.60, 0.30];
    c_text_muted    = [0.40, 0.40, 0.40];
    c_terminal_bg   = [0.90, 0.90, 0.92];
    c_terminal_txt  = [0.30, 0.30, 0.30];

    dim_pantalla = get(0, 'ScreenSize');

    % ---------------------------------------------------------------------
    % VENTANA PRINCIPAL
    % ---------------------------------------------------------------------
    fig = figure('Name', 'UNIVERSIDAD AUTÓNOMA DE QUERÉTARO — ENTORNO DE SIMULACIÓN BI-RRT*', ...
                 'Position', dim_pantalla, ...
                 'MenuBar', 'none', ...
                 'NumberTitle', 'off', ...
                 'Color', c_bg_lienzo);

    % ---------------------------------------------------------------------
    % BANNER SUPERIOR
    % ---------------------------------------------------------------------
    p_header = uipanel('Parent', fig, 'Title', '', ...
                       'Units', 'normalized', 'Position', [0.01, 0.91, 0.98, 0.07], ...
                       'BackgroundColor', c_uaq_azul, 'BorderType', 'etchedout');

    uicontrol('Parent', p_header, 'Style', 'text', ...
              'String', 'SISTEMA INTEGRADO DE ENRUTAMIENTO AUTÓNOMO EN MALLAS URBANAS COMPLEJAS', ...
              'Units', 'normalized', 'Position', [0.02, 0.50, 0.96, 0.45], ...
              'FontSize', 13, 'FontWeight', 'bold', 'HorizontalAlignment', 'left', ...
              'ForegroundColor', c_uaq_oro, 'BackgroundColor', c_uaq_azul);

    uicontrol('Parent', p_header, 'Style', 'text', ...
              'String', 'Plataforma Computacional de Optimización Algorítmica con Selector de Evolución Histórica', ...
              'Units', 'normalized', 'Position', [0.02, 0.05, 0.96, 0.40], ...
              'FontSize', 10, 'FontAngle', 'italic', 'HorizontalAlignment', 'left', ...
              'ForegroundColor', c_text_light, 'BackgroundColor', c_uaq_azul);

    % ---------------------------------------------------------------------
    % CONSOLA DE CONTROL LATERAL
    % ---------------------------------------------------------------------
    p_control = uipanel('Parent', fig, 'Title', '', ...
                        'Units', 'normalized', 'Position', [0.01, 0.02, 0.24, 0.88], ...
                        'BackgroundColor', c_panel_lateral, 'BorderType', 'etchedin');

    lbl_status = uicontrol('Parent', p_control, 'Style', 'text', ...
              'String', 'SISTEMA: LISTO PARA INICIALIZAR', ...
              'Units', 'normalized', 'Position', [0.05, 0.91, 0.90, 0.06], ...
              'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
              'ForegroundColor', c_uaq_oro, 'BackgroundColor', c_uaq_azul);

    % --- SECCIÓN I ---
    uicontrol('Parent', p_control, 'Style', 'text', 'String', 'I. ADQUISICIÓN CARTOGRÁFICA', ...
              'Units', 'normalized', 'Position', [0.05, 0.84, 0.90, 0.03], ...
              'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'left', ...
              'ForegroundColor', c_uaq_oro, 'BackgroundColor', c_panel_lateral);

    uicontrol('Parent', p_control, 'Style', 'pushbutton', 'String', 'Cargar Mapa del Repositorio', ...
              'Units', 'normalized', 'Position', [0.05, 0.77, 0.90, 0.06], ...
              'FontSize', 10, 'FontWeight', 'bold', ...
              'BackgroundColor', c_btn_base, 'ForegroundColor', c_text_light, ...
              'Callback', @analizarMapa);

    % --- SECCIÓN II ---
    uicontrol('Parent', p_control, 'Style', 'text', 'String', 'II. CONDICIONES DE CONTORNO', ...
              'Units', 'normalized', 'Position', [0.05, 0.70, 0.90, 0.03], ...
              'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'left', ...
              'ForegroundColor', c_uaq_oro, 'BackgroundColor', c_panel_lateral);

    uicontrol('Parent', p_control, 'Style', 'pushbutton', 'String', 'Fijar Vector de Origen', ...
              'Units', 'normalized', 'Position', [0.05, 0.63, 0.90, 0.06], ...
              'FontSize', 10, 'BackgroundColor', c_btn_base, 'ForegroundColor', c_text_light, ...
              'Callback', @seleccionarInicio);

    uicontrol('Parent', p_control, 'Style', 'pushbutton', 'String', 'Fijar Vector de Destino', ...
              'Units', 'normalized', 'Position', [0.05, 0.56, 0.90, 0.06], ...
              'FontSize', 10, 'BackgroundColor', c_btn_base, 'ForegroundColor', c_text_light, ...
              'Callback', @seleccionarFin);

    % --- SECCIÓN III: SELECTOR DE VERSIONES Y NÚCLEO ---
    uicontrol('Parent', p_control, 'Style', 'text', 'String', 'III. PROCESAMIENTO Y COMPARATIVA', ...
              'Units', 'normalized', 'Position', [0.05, 0.48, 0.90, 0.03], ...
              'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'left', ...
              'ForegroundColor', c_uaq_oro, 'BackgroundColor', c_panel_lateral);

    uicontrol('Parent', p_control, 'Style', 'text', 'String', 'Seleccione la Arquitectura a Ejecutar:', ...
              'Units', 'normalized', 'Position', [0.05, 0.42, 0.90, 0.03], ...
              'FontSize', 9, 'HorizontalAlignment', 'left', ...
              'ForegroundColor', c_text_light, 'BackgroundColor', c_panel_lateral);

    % MENÚ DESPLEGABLE DE VERSIONES
    drop_versiones = uicontrol('Parent', p_control, 'Style', 'popupmenu', ...
              'String', {'1. Bosque Bi-RRT* ', '2. Bi-RRT* Clásico (Bidireccional)', '3. RRT* Estándar (Crecimiento Único)', '4. RRT Voraz (Alta Dir. a Meta)', '5. RRT Búsqueda Binaria'}, ...
              'Units', 'normalized', 'Position', [0.05, 0.38, 0.90, 0.04], ...
              'FontSize', 9, 'BackgroundColor', c_bg_lienzo, 'ForegroundColor', [0.1 0.1 0.1]);

    uicontrol('Parent', p_control, 'Style', 'pushbutton', 'String', 'EJECUTAR OPTIMIZACIÓN', ...
              'Units', 'normalized', 'Position', [0.05, 0.25, 0.90, 0.10], ...
              'FontSize', 11, 'FontWeight', 'bold', ...
              'BackgroundColor', c_btn_action, 'ForegroundColor', c_text_light, ...
              'Callback', @ejecutarRRT);

    uicontrol('Parent', p_control, 'Style', 'pushbutton', 'String', '📊 COMPARAR MÉTRICAS (DASHBOARD)', ...
              'Units', 'normalized', 'Position', [0.05, 0.14, 0.90, 0.09], ...
              'FontSize', 10, 'FontWeight', 'bold', ...
              'BackgroundColor', c_uaq_oro, 'ForegroundColor', c_uaq_azul, ...
              'Callback', @(~,~) Comparar_Metricas_Mapa());

    % --- MANTENIMIENTO ---
    uicontrol('Parent', p_control, 'Style', 'pushbutton', 'String', 'Purgar Variables y Sistema', ...
              'Units', 'normalized', 'Position', [0.05, 0.05, 0.90, 0.06], ...
              'FontSize', 10, 'BackgroundColor', c_btn_reset, 'ForegroundColor', c_text_light, ...
              'Callback', @reiniciarSistema);

    % ---------------------------------------------------------------------
    % LIENZO DE VISUALIZACIÓN CARTOGRÁFICA
    % ---------------------------------------------------------------------
    p_mapa = uipanel('Parent', fig, 'Title', '', ...
                     'Units', 'normalized', 'Position', [0.26, 0.02, 0.73, 0.88], ...
                     'BackgroundColor', c_bg_lienzo, 'BorderType', 'none');

    ax = axes('Parent', p_mapa, 'Units', 'normalized', 'Position', [0.08, 0.08, 0.88, 0.84], ...
              'Box', 'on', 'LineWidth', 1, 'XColor', c_text_muted, 'YColor', c_text_muted);

    set(ax, 'Color', c_terminal_bg);

    title(ax, 'Área de Despliegue Vectorial Espacial (Resolución Escalar 500x500 px)', ...
          'FontSize', 11, 'FontWeight', 'bold', 'Color', c_uaq_azul);
    xlabel(ax, 'Eje Geométrico X (Metros)', 'FontSize', 9);
    ylabel(ax, 'Eje Geométrico Y (Metros)', 'FontSize', 9);
    set(ax, 'XLim', [0 500], 'YLim', [0 500]);
    axis(ax, 'equal'); grid(ax, 'on');
    set(ax, 'GridColor', [0.4, 0.4, 0.4], 'GridAlpha', 0.5);
    hold(ax, 'on');


    % =====================================================================
    % SUBSISTEMAS LÓGICOS DE OPERACIÓN
    % =====================================================================

    function analizarMapa(~, ~)
        set(lbl_status, 'String', 'RUN: ESCANEANDO REPOSITORIO LOCAL...', 'ForegroundColor', c_uaq_oro);
        drawnow;

        carpeta_mapas = 'Mapas-OSM';

        if ~exist(carpeta_mapas, 'dir')
            errordlg(['No se encontró la carpeta "', carpeta_mapas, '". Créala en la misma ubicación que este script.'], 'Error de Directorio');
            set(lbl_status, 'String', 'ERROR: REPOSITORIO NO ENCONTRADO', 'ForegroundColor', [1.0, 0.3, 0.3]);
            return;
        end

        lista_archivos = dir(fullfile(carpeta_mapas, '*.osm'));

        if isempty(lista_archivos)
            errordlg(['La carpeta "', carpeta_mapas, '" está vacía. Añade tus archivos .osm primero.'], 'Repositorio Vacío');
            return;
        end

        nombres_mapas = {lista_archivos.name};

        [idx, ok] = listdlg('PromptString', 'Seleccione una malla vectorial:', ...
                            'Name', 'Repositorio Mapas-OSM', 'SelectionMode', 'single', ...
                            'ListSize', [250, 150], 'ListString', nombres_mapas);

        if ok
            archivo = nombres_mapas{idx};
            mapa_actual_nombre = archivo;
            ruta_absoluta = fullfile(pwd, carpeta_mapas);

            set(lbl_status, 'String', 'SYS: EJECUTANDO PIPELINE EN PYTHON...', 'ForegroundColor', c_text_light);
            drawnow;

            comando = sprintf('py analizar_osm.py "%s"', fullfile(ruta_absoluta, archivo));
            [status, out] = system(comando);
            disp(out);

            if status == 0
                evalin('base', 'clear Mapa_Obstaculos_RRT');
                evalin('base', 'Mapa_Obstaculos_RRT');
                obstaculos = evalin('base', 'obstaculos');
                mapa_listo = true;

                set(lbl_status, 'String', 'SYS: CACHING DE MATRICES A RAM...', 'ForegroundColor', c_text_light);
                drawnow;
                disp('Cargando mapas digitales en la RAM...');

                mapa_esqueleto = load('Mapa_Esqueleto.txt');
                [filas_esq, cols_esq] = find(mapa_esqueleto == 1);
                coordenadas_esqueleto = [cols_esq - 1, filas_esq - 1];
                mapa_discreto = load('Mapa_Discreto.txt');

                set(lbl_status, 'String', 'SYS: CALCULANDO TEOREMA DE MALLA...', 'ForegroundColor', c_text_light);
                drawnow;
                disp('Generando semillas uniformes...');

                semillas = Generar_Semillas_Malla(coordenadas_esqueleto, 50);

                punto_inicio = [];
                punto_final = [];

                cla(ax, 'reset'); hold(ax, 'on');

                color_manzana = [0.28, 0.31, 0.36];
                for k = 1:size(obstaculos, 1)
                    patch(obstaculos{k,1}(:,1), obstaculos{k,1}(:,2), color_manzana, ...
                          'Parent', ax, 'EdgeColor', [0.10, 0.12, 0.15], 'LineWidth', 0.8);
                end

                set(ax, 'XLim', [0 500], 'YLim', [0 500]);
                axis(ax, 'equal'); grid(ax, 'on');
                title(ax, ['Topología Urbana Indexada: ', archivo], 'Color', c_uaq_azul, 'FontSize', 11, 'FontWeight', 'bold');

                set(lbl_status, 'String', 'OK: ENTORNO CARGADO. FIJE ORIGEN', 'ForegroundColor', [0.3, 0.9, 0.5]);
            else
                set(lbl_status, 'String', 'ERROR: FALLO EN PREPROCESAMIENTO', 'ForegroundColor', [1.0, 0.3, 0.3]);
            end
        else
            set(lbl_status, 'String', 'IDLE: IMPORTACIÓN CANCELADA', 'ForegroundColor', c_text_muted);
        end
    end

    function seleccionarInicio(~, ~)
        if ~mapa_listo
            return;
        end
        punto_final = [];
        cla(ax, 'reset'); hold(ax, 'on');

        color_manzana = [0.28, 0.31, 0.36];
        for k = 1:size(obstaculos, 1)
            patch(obstaculos{k,1}(:,1), obstaculos{k,1}(:,2), color_manzana, 'Parent', ax, 'EdgeColor', [0.10, 0.12, 0.15]);
        end

        set(ax, 'XLim', [0 500], 'YLim', [0 500]);
        axis(ax, 'equal'); grid(ax, 'on');

        title(ax, 'MODO CAPTURA: Capture el Vector de Origen', 'Color', c_btn_reset, 'FontSize', 11, 'FontWeight', 'bold');
        set(lbl_status, 'String', 'INPUT: ESPERANDO CLIC DE ORIGEN...', 'ForegroundColor', c_uaq_oro);
        axes(ax);
        [ix, iy] = ginput(1);

        dentro_de_obstaculo = false;
        for k = 1:size(obstaculos, 1)
            poligono = obstaculos{k, 1};
            if inpolygon(ix, iy, poligono(:,1), poligono(:,2))
                dentro_de_obstaculo = true;
                break;
            end
        end

        if dentro_de_obstaculo
            set(lbl_status, 'String', 'ERROR: INTERCEPCIÓN EN ORIGEN', 'ForegroundColor', [1.0, 0.3, 0.3]);
            return;
        end

        punto_inicio = [ix, iy];
        plot(ax, ix, iy, 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.0, 0.7, 0.4], 'MarkerEdgeColor', 'white', 'LineWidth', 1.2);
        title(ax, 'Vector Origen registrado en base de datos.', 'Color', c_uaq_azul, 'FontSize', 11);
        set(lbl_status, 'String', 'OK: ORIGEN FIJADO. FIJE DESTINO', 'ForegroundColor', c_text_light);
    end

    function seleccionarFin(~, ~)
        if ~mapa_listo || isempty(punto_inicio)
            return;
        end

        cla(ax, 'reset'); hold(ax, 'on');

        color_manzana = [0.28, 0.31, 0.36];
        for k = 1:size(obstaculos, 1)
            patch(obstaculos{k,1}(:,1), obstaculos{k,1}(:,2), color_manzana, 'Parent', ax, 'EdgeColor', [0.10, 0.12, 0.15]);
        end

        plot(ax, punto_inicio(1), punto_inicio(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.0, 0.7, 0.4], 'MarkerEdgeColor', 'white', 'LineWidth', 1.2);

        set(ax, 'XLim', [0 500], 'YLim', [0 500]);
        axis(ax, 'equal'); grid(ax, 'on');

        title(ax, 'MODO CAPTURA: Capture el Vector de Destino', 'Color', c_btn_reset, 'FontSize', 11, 'FontWeight', 'bold');
        set(lbl_status, 'String', 'INPUT: ESPERANDO CLIC DE DESTINO...', 'ForegroundColor', c_uaq_oro);
        axes(ax);
        [ox, oy] = ginput(1);

        dentro_de_obstaculo = false;
        for k = 1:size(obstaculos, 1)
            poligono = obstaculos{k, 1};
            if inpolygon(ox, oy, poligono(:,1), poligono(:,2))
                dentro_de_obstaculo = true;
                break;
            end
        end

        if dentro_de_obstaculo
            set(lbl_status, 'String', 'ERROR: INTERCEPCIÓN EN DESTINO', 'ForegroundColor', [1.0, 0.3, 0.3]);
            return;
        end

        punto_final = [ox, oy];
        plot(ax, ox, oy, 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.9, 0.1, 0.2], 'MarkerEdgeColor', 'white', 'LineWidth', 1.2);
        title(ax, 'Estructura de nodos lista para computación.', 'Color', c_uaq_azul, 'FontSize', 11, 'FontWeight', 'bold');
        set(lbl_status, 'String', 'OK: SISTEMA PREPARADO PARA RUTEO', 'ForegroundColor', [0.3, 0.9, 0.5]);
    end

    function ejecutarRRT(~, ~)
        if isempty(punto_inicio) || isempty(punto_final)
            errordlg('Falta de parámetros cinemáticos de entrada.', 'Inviabilidad de Ejecución');
            return;
        end

        % LEER EL MENÚ DESPLEGABLE
        version_seleccionada = get(drop_versiones, 'Value');

        cla(ax, 'reset'); hold(ax, 'on');

        color_manzana = [0.28, 0.31, 0.36];
        for k = 1:size(obstaculos, 1)
            patch(obstaculos{k,1}(:,1), obstaculos{k,1}(:,2), color_manzana, 'Parent', ax, 'EdgeColor', [0.10, 0.12, 0.15]);
        end

        plot(ax, punto_inicio(1), punto_inicio(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.0, 0.7, 0.4], 'MarkerEdgeColor', 'white', 'LineWidth', 1.2);
        plot(ax, punto_final(1), punto_final(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.9, 0.1, 0.2], 'MarkerEdgeColor', 'white', 'LineWidth', 1.2);

        set(ax, 'XLim', [0 500], 'YLim', [0 500]);
        axis(ax, 'equal'); grid(ax, 'on');

        title(ax, ['CÓMPUTO CRÍTICO: Ejecutando Variante Histórica ', num2str(version_seleccionada)], 'Color', c_uaq_azul, 'FontSize', 11, 'FontWeight', 'bold');
        set(lbl_status, 'String', 'RUN: PROCESANDO HEURÍSTICA...', 'ForegroundColor', c_btn_run);
        drawnow;

        % ENRUTADOR MODULAR: Llama a un script distinto según la versión
        switch version_seleccionada
            case 1
                disp('>>> Ejecutando V36 (Bosque Bi-RRT* Final)...');
                Ejecutar_RRT_Star;
            case 2
                disp('>>> Ejecutando V20 (Bi-RRT* Mejorado)...');
                Ejecutar_Bi_RRT_Clasico;
            case 3
                disp('>>> Ejecutando V15 (Bi-RRT* Estándar)...');
                Ejecutar_RRT_Estandar;
            case 4
                disp('>>> Ejecutando V10 (RRT Voraz)...');
                Ejecutar_RRT_Voraz;
            case 5
                disp('>>> Ejecutando V05 (RRT Búsqueda Binaria)...');
                Ejecutar_RRT_Binaria_V5;;
        end

        % --- ETIQUETADO POST-EJECUCIÓN DEL MAPA ---
        try
            if ~strcmp(mapa_actual_nombre, 'Sin_Asignar')
                ruta_res = fullfile(pwd, 'Resultados_Metricas');
                lista = dir(fullfile(ruta_res, '*.txt'));

                if ~isempty(lista)
                    [~, idx_reciente] = max([lista.datenum]);
                    archivo_reciente = fullfile(ruta_res, lista(idx_reciente).name);

                    fid = fopen(archivo_reciente, 'a');
                    if fid ~= -1
                        fprintf(fid, 'Mapa: %s\n', mapa_actual_nombre);
                        fclose(fid);
                        disp(['>>> [Sistema] Etiqueta de mapa agregada con éxito en: ', archivo_reciente]);
                    end
                end
            end
        catch ME
            disp(['>>> [Sistema] Nota: No se pudo etiquetar el mapa: ', ME.message]);
        end
        % ----------------------------------------------------------

        title(ax, 'Análisis Finalizado — Trayectoria Determinada', 'Color', c_uaq_azul, 'FontSize', 11, 'FontWeight', 'bold');
        set(lbl_status, 'String', 'OK: PROCESO FINALIZADO', 'ForegroundColor', [0.3, 0.9, 0.5]);
    end

    function reiniciarSistema(~, ~)
        punto_inicio = []; punto_final = []; obstaculos = {};
        mapa_esqueleto = []; coordenadas_esqueleto = [];
        mapa_discreto = []; semillas = []; mapa_listo = false;

        cla(ax, 'reset');
        title(ax, 'Estructuras GUI inicializadas. Esperando nueva matriz de datos.', 'Color', c_uaq_azul, 'FontSize', 11, 'FontWeight', 'bold');
        set(ax, 'XLim', [0 500], 'YLim', [0 500]); grid(ax, 'on');
        set(lbl_status, 'String', 'IDLE: ESPERANDO CARGA DE MALLA VECTORIAL.', 'ForegroundColor', c_terminal_txt);
        disp('>>> Registros internos borrados de forma segura.');
    end
end
