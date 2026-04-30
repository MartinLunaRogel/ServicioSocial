% =========================================================================
% === INTERFAZ GRÁFICA PRINCIPAL DEL NAVEGADOR RRT* ===

% Archivo de ejecucion principal. Crea el menu en una ventana , dubuja los botones
% y conecta cada boton con sus tareas respectivas.
% Carga el mapa, Eleguir punto de inicio y objetivo dentro del mapa, busca la ruta
% y reinicia el programa.
% Esta diseñado para interactual visualemnte con el programa.
% =========================================================================

function Interfaz_RRT()

    % Cambiamos la carpeta de trabajo a la ruta exacta donde está guardado este archivo
    cd(fileparts(mfilename('fullpath')));

    % Limpiar variables globales por si ejecutaste el programa antes
    clear global punto_inicio punto_final obstaculos mapa_esqueleto coordenadas_esqueleto mapa_discreto semillas;

    % Declaramos cuáles variables se compartirán con los demás archivos (variables globales)
    global obstaculos punto_inicio punto_final mapa_listo mapa_esqueleto coordenadas_esqueleto mapa_discreto semillas;

    mapa_listo = false;   % Indicador de cuando ya esta cargado un mapa

     % ---  CREACIÓN DE LA VENTANA Y EL MAPA ---
    fig = figure('Name', 'Navegador RRT*', ...      % Titulo de la ventana
                 'Position', [100 100 800 600], ... % tamaño inicial (al abrirse la ventana en automatico)
                 'MenuBar', 'none', ...             % Se ocultan las barras del menu
                 'NumberTitle', 'off');

    % Creamos el espacio (ejes) donde se dibujará el mapa, indicando que ocupará el 70% derecho de la ventana
    ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.25 0.05 0.70 0.90]);
    title(ax, 'Mapa de Obstáculos');                % Titulo de la grafica principal
    xlabel(ax, 'X'); ylabel(ax, 'Y');               % Etiqueta de los ejes X y Y
    set(ax, 'XLim', [0 500], 'YLim', [0 500]);      % Se dibuja de 0 a 500 en ambos ejes
    axis(ax, 'equal');                              % Evita que el mapa se deforme al maximizar
    grid(ax, 'on');                                 % Cuadricula de fondo
    hold(ax, 'on');                                 % hace que se mantengan todos los dibujos al dubujar uno nuevo

    % --- CREACIÓN DEL PANEL LATERAL DE BOTONES ---

    % Crea un texto estático que dice "CONTROLES" para que sirva como título de los botones
    uicontrol('Parent', fig, 'Style', 'text', 'String', 'CONTROLES', ...
              'Units', 'normalized', 'Position', [0.02 0.85 0.20 0.05], ...
              'FontSize', 12, 'FontWeight', 'bold', 'BackgroundColor', get(fig, 'Color'));

    % Botón 1: Ejecutará la función interna @analizarMapa cuando el usuario le haga clic
    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '1. Analizar Mapa', ...
              'Units', 'normalized', 'Position', [0.02 0.75 0.20 0.08], 'Callback', @analizarMapa);

    % Botón 2: Ejecutará la función interna @seleccionarInicio cuando el usuario le haga clic
    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '2. Punto Inicio', ...
              'Units', 'normalized', 'Position', [0.02 0.65 0.20 0.08], 'Callback', @seleccionarInicio);

    % Botón 3: Ejecutará la función interna @seleccionarFin cuando el usuario le haga clic
    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '3. Punto Fin', ...
              'Units', 'normalized', 'Position', [0.02 0.55 0.20 0.08], 'Callback', @seleccionarFin);

    % Botón 4: Ejecutará @ejecutarRRT, lo pintamos de un color verde claro para indicar "Acción/Inicio"
    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '4. Iniciar Búsqueda', ...
              'Units', 'normalized', 'Position', [0.02 0.45 0.20 0.08], ...
              'BackgroundColor', [0.8 1 0.8], 'Callback', @ejecutarRRT);

    % Botón 5: Ejecutará @reiniciarSistema, lo pintamos de color rojizo para indicar "Borrado/Reinicio"
    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '5. Reiniciar Sistema', ...
              'Units', 'normalized', 'Position', [0.02 0.35 0.20 0.08], ...
              'BackgroundColor', [1 0.8 0.8], 'Callback', @reiniciarSistema);

    % =====================================================================
    % === LÓGICA DE CADA BOTÓN (FUNCIONES INTERNAS) ===
    % =====================================================================

    function analizarMapa(~, ~)                     % Funcion para cargar y analizar un nuevo mapa
        [archivo, ruta] = uigetfile('*.osm', 'Selecciona el mapa OSM');  % Se abre el explorador de archivos para seleccionar un .OSM
        if archivo ~= 0                             % Si se selecciono un archivo continua
            comando = sprintf('py analizar_osm.py "%s"', fullfile(ruta, archivo));  % Se construye un comando de texto para mandar
                                                                                    % a llamar el script de pyton analizar_osm.py

            [status, out] = system(comando);        % Ejecutamos el comando en la terminal del sistema y esperamos a que termine
            disp(out);                              % Se muestra en la consola lo que imprimio python para saber que pasó
            if status == 0                          % Si python termino exitosamente = "0", continua
                evalin('base', 'clear Mapa_Obstaculos_RRT');  % Le decimos a la memoria base de Octave que borre mapas pasados
                evalin('base', 'Mapa_Obstaculos_RRT');        % Le decimos a la memoria base que lea el archivo .m recién generado por Python
                obstaculos = evalin('base', 'obstaculos');    % Se toma la variable 'obstaculos' que se generó y se muetra en la interfaz
                mapa_listo = true;                  % Se cambia el candado de seguridad, el mapa ya esta listo


                % =========================================================
                % --- CACHING EN MEMORIA RAM ---
                disp('Cargando mapas digitales en la RAM...');        % Mensaje para el usuario en consola
                mapa_esqueleto = load('Mapa_Esqueleto.txt');          % Se carga la matriz del esqueleto de calles desde el archivo de texto
                [filas_esq, cols_esq] = find(mapa_esqueleto == 1);    % Se busca en qué filas y columnas están los '1' (las calles libres)
                coordenadas_esqueleto = [cols_esq - 1, filas_esq - 1];% Convertimos esos índices a coordenadas (X, Y)

                mapa_discreto = load('Mapa_Discreto.txt');            % Cargamos la matriz de edificios (1=bloqueo, 0=libre) usada para colisiones

                disp('Generando semillas uniformes...');              % Mensaje informativo en consola
                semillas = Generar_Semillas_Malla(coordenadas_esqueleto, 50); % Se calculan las semillas esparcidos por el mapa, separados cada 50 unidades
                % =========================================================

                % Se corran las coordenadas viejas de búsquedas anteriores para evitar conflictos
                punto_inicio = [];
                punto_final = [];

                cla(ax);                            % Limpiar y dibujar solo el mapa
                hold(ax, 'on');                     % Se activa la retención del lienzo

                % Iniciamos un ciclo para dibujar cada edificio gris (obstáculo) uno por uno
                for k = 1:size(obstaculos, 1)
                    % Se dubuja el polígono en el mapa en gris oscuro
                    patch(obstaculos{k,1}(:,1), obstaculos{k,1}(:,2), [0.3 0.3 0.3], 'Parent', ax);
                end

                set(ax, 'XLim', [0 500], 'YLim', [0 500]);    % Re-aplicamos los límites del mapa para asegurarnos de que la cámara no se haya movido
                axis(ax, 'equal'); grid(ax, 'on');            % Mantenemos la proporción cuadrada de los ejes
                msgbox('Mapa cargado y procesado.', 'Éxito'); % Mostramos un cuadro de diálogo avisando al usuario que todo salió bien
            end
        end
    end

    function seleccionarInicio(~, ~)    % Funcion para poner el punto inicial en el mapa
        if ~mapa_listo                  % Si el usuario no ha cargado un mapa, bloqueamos la acción con un mensaje de error
            errordlg('Carga el mapa primero.', 'Error'); return;
        end

        % Borramos la meta anterior, ya que al mover el inicio la ruta vieja queda inválida
        punto_final = [];

        % Borramos todo el contenido visual del mapa (árboles viejos, rutas, etc.)
        cla(ax); hold(ax, 'on');

        % Redibujamos los edificios de color gris oscuro en el mapa limpio
        for k = 1:size(obstaculos, 1)
            patch(obstaculos{k,1}(:,1), obstaculos{k,1}(:,2), [0.3 0.3 0.3], 'Parent', ax);
        end

        set(ax, 'XLim', [0 500], 'YLim', [0 500]);          % Restauramos la configuración de vista del mapa
        axis(ax, 'equal'); grid(ax, 'on');

        title(ax, 'Haz clic en el mapa para el INICIO');    % Cambiamos el título para darle instrucciones al usuario
        axes(ax);                                           % Le indicamos al programa que enfoque su atención en nuestro lienzo principal
        [ix, iy] = ginput(1);                               % Congelamos el programa hasta que el usuario dé 1 clic en el mapa, y guardamos la coordenada (X,Y)

        % --- VALIDACIÓN DE COLISIÓN DEL MOUSE ---
        % Creamos un interruptor apagado que indica si el clic fue dentro de un obstaculo
        dentro_de_obstaculo = false;

        for k = 1:size(obstaculos, 1)                       % Revisamos obstaculo por obstaculo
            poligono = obstaculos{k, 1};                    % Extraemos las esquinas de la manzana actual
            if inpolygon(ix, iy, poligono(:,1), poligono(:,2))  % Revisamos si la coordenada del clic (ix, iy) cayó dentro del área de este obstaculo
                dentro_de_obstaculo = true;                 % Si cayó dentro, encendemos la alarma y detenemos la búsqueda de obstaculos
                break;
            end
        end

        if dentro_de_obstaculo                                      % Si el usuario hizo clic en un lugar prohibido (obstaculo)
            % Le mostramos un error de Windows advirtiendo la situación
            errordlg('El punto está dentro de un area indevida. Por favor, selecciona una calle libre.', 'Punto Inválido');
            title(ax, 'Error: Selecciona el inicio en una calle.'); % Cambiamos el título del mapa indicando el fallo
            return;                                                 % Cancelamos la acción y no guardamos el punto
        end
        % =========================================================

        punto_inicio = [ix, iy];                              % Si el punto es válido, lo guardamos en nuestra variable global
        plot(ax, ix, iy, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Dibujamos un círculo verde ('go') en las coordenadas dadas, con tamaño 10
        title(ax, 'Inicio fijado. Selecciona el Objetivo.');  % Actualizamos el título para indicar qué sigue
    end


    function seleccionarFin(~, ~)                   % funcion para seleccionar el punto de meta (objetivo)

        if ~mapa_listo                              % Si no hay mapa, mostramos error y cancelamos
            errordlg('Carga el mapa primero.', 'Error'); return;
        end

        % Si el usuario intenta poner el Fin sin haber puesto el Inicio, mostramos error
        if isempty(punto_inicio)
            errordlg('Selecciona el punto de INICIO primero.', 'Error'); return;
        end

        % Limpiamos el mapa de rutas viejas
        cla(ax); hold(ax, 'on');

        % Redibujamos los obstáculos grises
        for k = 1:size(obstaculos, 1)
            patch(obstaculos{k,1}(:,1), obstaculos{k,1}(:,2), [0.3 0.3 0.3], 'Parent', ax);
        end

        % Redibujamos el punto Verde del Inicio para que no desaparezca
        plot(ax, punto_inicio(1), punto_inicio(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

        set(ax, 'XLim', [0 500], 'YLim', [0 500]);          % Restauramos la configuración de los ejes
        axis(ax, 'equal'); grid(ax, 'on');

        title(ax, 'Haz clic en el mapa para el OBJETIVO');  % Instruimos al usuario a elegir la meta
        axes(ax);                                           % Enfocamos el lienzo
        [ox, oy] = ginput(1);                               % Capturamos 1 clic con el mouse para el objetivo (ox, oy)

        % =========================================================
        % --- VALIDACIÓN DE COLISIÓN DEL MOUSE ---
        dentro_de_obstaculo = false;                        % Interruptor de alarma
        for k = 1:size(obstaculos, 1)                       % Revisamos si el clic tocó algún edificio
            poligono = obstaculos{k, 1};
            if inpolygon(ox, oy, poligono(:,1), poligono(:,2))
                dentro_de_obstaculo = true;
                break;
            end
        end

        if dentro_de_obstaculo                              % Si el clic fue en lugar prohibido, mandamos error y abortamos
            errordlg('El objetivo está dentro de un area indevida. Por favor, selecciona una calle libre.', 'Punto Inválido');
            title(ax, 'Error: Selecciona la meta en una calle.');
            return; % Cancelamos la acción y no guardamos el punto
        end
        % =========================================================

        % Guardamos el punto válido en la variable global
        punto_final = [ox, oy];
        plot(ax, ox, oy, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Dibujamos el punto en el mapa como un círculo rojo ('ro') relleno
        title(ax, 'Objetivo fijado. Listo para buscar.');                 % Actualizamos el título indicando que el sistema ya puede arrancar
    end

    function ejecutarRRT(~, ~)                    % Funcion para ejecutar la busqueda del algoritmo RRT y buscar el mejor camino

        % Verificamos que los dos puntos existan. Si falta alguno, cancelamos el arranque
        if isempty(punto_inicio) || isempty(punto_final)
            errordlg('Selecciona inicio y fin primero.', 'Error'); return;
        end

        % =====================================================================
        % === REFRESCAR EL MAPA ANTES DE EMPEZAR ===
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

        title(ax, 'Calculando nueva ruta...');  % Cambiamos el texto de la pantalla para informar que el sistema está trabajando
        axes(ax);                               % Asegurar que el algoritmo dibuje en el cuadro correcto

        % Llamamos y ejecutamos el script principal del algoritmo
        Ejecutar_RRT_Star;

        % Cuando termina el algoritmo, cambiamos el título final
        title(ax, 'Búsqueda Finalizada');
    end


    function reiniciarSistema(~, ~)       % Función que Reinicia el sistema

        % Vaciado seguro de variables globales sin destruir su estructura en memoria
        punto_inicio = [];
        punto_final = [];
        obstaculos = {};
        mapa_esqueleto = [];
        coordenadas_esqueleto = [];
        mapa_discreto = [];
        semillas = [];

        % Bloqueo del sistema indicando ausencia de mapa
        mapa_listo = false;

        % Limpieza visual del área del mapa
        cla(ax);
        title(ax, 'Sistema Reiniciado. Carga un mapa para comenzar.');
        set(ax, 'XLim', [0 500], 'YLim', [0 500]);
        grid(ax, 'on');

        % Emisión de mensaje técnico en consola
        disp('>>> Memoria limpia y sistema reiniciado.');

        % Despliegue de confirmación visual para el usuario
        msgbox('El sistema se ha reiniciado correctamente.', 'Reinicio');
    end
end
