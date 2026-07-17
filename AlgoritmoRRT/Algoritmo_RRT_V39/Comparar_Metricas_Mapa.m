% =========================================================================
% === MÓDULO INSTITUCIONAL DE COMPARATIVA Y BENCHMARKING POR MAPA ===
%
% Escanea los reportes generados en la carpeta central 'Resultados_Metricas',
% agrupa por mapa vectorial, advierte sobre ejecuciones faltantes y despliega
% un dashboard de 4 subplots con paleta UAQ para la defensa académica.
% =========================================================================

function Comparar_Metricas_Mapa()

    % 1. Localizar archivos .txt en la carpeta única central
    ruta_unica = fullfile(pwd, 'Resultados_Metricas');

    rutas_archivos = {};
    fechas_archivos = [];
    nombres_archivos = {};

    if exist(ruta_unica, 'dir')
        lista = dir(fullfile(ruta_unica, '*.txt'));
        for k = 1:length(lista)
            rutas_archivos{end+1} = fullfile(ruta_unica, lista(k).name);
            fechas_archivos(end+1) = lista(k).datenum;
            nombres_archivos{end+1} = lista(k).name;
        end
    end

    if isempty(rutas_archivos)
        errordlg('No se han encontrado archivos de métricas generados. Ejecute al menos una versión en la interfaz primero.', 'Repositorio Vacío');
        return;
    end

    % 2. Parsear el contenido de todos los reportes encontrados
    datos_reportes = struct();
    num_validos = 0;

    for i = 1:length(rutas_archivos)
        ruta_completa = rutas_archivos{i};
        texto = fileread(ruta_completa);
        nombre_arch = nombres_archivos{i};

        % Identificar versión del algoritmo
        ver_idx = 0;
        if ~isempty(regexpi(texto, 'V36|V38|Bosque')) || ~isempty(regexpi(nombre_arch, 'V36|V38'))
            ver_idx = 1; % V38/V36
        elseif ~isempty(regexpi(texto, 'V20')) || ~isempty(regexpi(nombre_arch, 'V20'))
            ver_idx = 2; % V20
        elseif ~isempty(regexpi(texto, 'V15')) || ~isempty(regexpi(nombre_arch, 'V15'))
            ver_idx = 3; % V15
        elseif ~isempty(regexpi(texto, 'V10|VORAZ')) || ~isempty(regexpi(nombre_arch, 'V10'))
            ver_idx = 4; % V10 Voraz
        elseif ~isempty(regexpi(texto, 'V5|BINARIA')) || ~isempty(regexpi(nombre_arch, 'V5'))
            ver_idx = 5; % V5 Binaria
        end

        if ver_idx == 0
            continue;
        end

        % Identificar mapa asociado
        tok_mapa = regexp(texto, 'Mapa:\s*([^\r\n]+)', 'tokens', 'once');
        if ~isempty(tok_mapa)
            nombre_mapa = strtrim(tok_mapa{1});
        else
            nombre_mapa = 'General / Sin Asignar';
        end

        % Extraer métricas numéricas por Regex
        t_busq = extraer_numero(texto, 'Tiempo\s*B[uú]squeda(\s*Inicial)?:\s*([\d\.]+)');
        t_opt  = extraer_numero(texto, 'Tiempo\s*Optimizaci[oó]n:\s*([\d\.]+)');
        c_fin  = extraer_numero(texto, 'Costo\s*(Final\s*de\s*)?Ruta:\s*([\d\.]+)');
        n_arb  = extraer_numero(texto, 'Nodos\s*totales\s*en\s*[\wáéíóú]+:\s*(\d+)');
        n_cam  = extraer_numero(texto, 'Nodos\s*en\s*camino\s*(final)?:\s*(\d+)');
        n_iter = extraer_numero(texto, 'Iteraciones:\s*(\d+)');
        n_col  = extraer_numero(texto, 'Colisiones:\s*(\d+)');
        efic   = extraer_numero(texto, 'Eficiencia\s*de\s*Muestreo:\s*([\d\.]+)');

        % Cálculo de respaldo para eficiencia si no estaba explícito
        if efic == 0 && n_iter > 0
            efic = (1 - (n_col / n_iter)) * 100;
        end

        num_validos = num_validos + 1;
        datos_reportes(num_validos).version_idx = ver_idx;
        datos_reportes(num_validos).mapa = nombre_mapa;
        datos_reportes(num_validos).t_busq = t_busq;
        datos_reportes(num_validos).t_opt = t_opt;
        datos_reportes(num_validos).costo = c_fin;
        datos_reportes(num_validos).n_arboles = n_arb;
        datos_reportes(num_validos).n_camino = n_cam;
        datos_reportes(num_validos).eficiencia = efic;
        datos_reportes(num_validos).datenum = fechas_archivos(i);
    end

    if num_validos == 0
        errordlg('Los archivos de texto encontrados no coinciden con el formato esperado del sistema.', 'Error de Lectura');
        return;
    end

    % 3. Selector interactivo de mapa
    lista_mapas = unique({datos_reportes.mapa});
    [idx_sel, ok] = listdlg('PromptString', 'Seleccione el mapa para generar el Benchmarking:', ...
                            'Name', 'Selector de Malla para Comparativa', ...
                            'SelectionMode', 'single', 'ListSize', [300, 150], ...
                            'ListString', lista_mapas);
    if ~ok
        return;
    end
    mapa_elegido = lista_mapas{idx_sel};

    % 4. Aislar la ejecución más reciente por versión para el mapa elegido
    nombres_versiones = {'1. Bosque Bi-RRT* (V38)', '2. Bi-RRT* Clásico (V20)', ...
                         '3. RRT* Estándar (V15)', '4. RRT Voraz (V10)', '5. RRT Búsqueda Binaria (V5)'};

    versiones_presentes = false(1, 5);
    metricas_finales = repmat(struct('t_busq', 0, 't_opt', 0, 'costo', 0, ...
                                     'n_arboles', 0, 'n_camino', 0, 'eficiencia', 0), 1, 5);

    for v = 1:5
        candidatos = [];
        for d = 1:num_validos
            if datos_reportes(d).version_idx == v && strcmp(datos_reportes(d).mapa, mapa_elegido)
                candidatos = [candidatos, d];
            end
        end
        if ~isempty(candidatos)
            % Tomar la ejecución más reciente en el tiempo
            [~, pos_reciente] = max([datos_reportes(candidatos).datenum]);
            win_idx = candidatos(pos_reciente);

            versiones_presentes(v) = true;

            % Asignación campo por campo para evitar incompatibilidad de estructuras en Octave
            metricas_finales(v).t_busq     = datos_reportes(win_idx).t_busq;
            metricas_finales(v).t_opt      = datos_reportes(win_idx).t_opt;
            metricas_finales(v).costo      = datos_reportes(win_idx).costo;
            metricas_finales(v).n_arboles  = datos_reportes(win_idx).n_arboles;
            metricas_finales(v).n_camino   = datos_reportes(win_idx).n_camino;
            metricas_finales(v).eficiencia = datos_reportes(win_idx).eficiencia;
        end
    end

    % 5. Auditoría y alerta de versiones faltantes
    if ~all(versiones_presentes)
        faltantes_str = strjoin(nombres_versiones(~versiones_presentes), '\n * ');
        mensaje_alerta = sprintf(['Aviso de Auditoría — Benchmarking Parcial\n\n', ...
                                  'Para el mapa "%s", aún falta ejecutar las siguientes arquitecturas:\n\n * %s\n\n', ...
                                  'Se generará el dashboard con las %d versión(es) disponible(s). ', ...
                                  'Ejecútelas en la interfaz y pulse comparar de nuevo para el reporte completo.'], ...
                                  mapa_elegido, faltantes_str, sum(versiones_presentes));
        warndlg(mensaje_alerta, 'Auditoría de Versiones Faltantes');
    end

    % 6. Renderizado del Dashboard Institucional UAQ (4 Gráficas)
    c_uaq_azul  = [0.03, 0.13, 0.27];
    c_uaq_oro   = [0.76, 0.61, 0.18];
    c_bg_lienzo = [0.96, 0.96, 0.97];
    c_verde_opt = [0.11, 0.41, 0.27];
    c_rojo_acc  = [0.70, 0.15, 0.15];
    c_gris_bar  = [0.40, 0.50, 0.60];

    fig_dash = figure('Name', ['BENCHMARKING INSTITUCIONAL UAQ — MAPA: ', upper(mapa_elegido)], ...
                      'Color', c_bg_lienzo, 'NumberTitle', 'off', ...
                      'Units', 'normalized', 'Position', [0.05, 0.05, 0.90, 0.88]);

    % Anotación de encabezado en la ventana
    annotation(fig_dash, 'textbox', [0.02, 0.94, 0.96, 0.05], ...
               'String', ['ANÁLISIS COMPARATIVO DE RENDIMIENTO ALGORÍTMICO — MALLA: ', mapa_elegido], ...
               'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
               'Color', c_uaq_oro, 'BackgroundColor', c_uaq_azul, 'EdgeColor', 'none');

    etiquetas_activas = nombres_versiones(versiones_presentes);
    m_act = metricas_finales(versiones_presentes);
    num_barras = length(m_act);
    x_vec = 1:num_barras;

    % --- SUBPLOT 1: TIEMPOS DE EJECUCIÓN ---
    ax1 = subplot(2, 2, 1, 'Parent', fig_dash);
    matriz_tiempos = [[m_act.t_busq]', [m_act.t_opt]'];
    b1 = bar(ax1, x_vec, matriz_tiempos, 'stacked');
    if length(b1) >= 1, set(b1(1), 'FaceColor', c_uaq_azul); end
    if length(b1) >= 2, set(b1(2), 'FaceColor', c_verde_opt); end
    title(ax1, 'I. Desglose de Tiempo de Ejecución (Segundos)', 'Color', c_uaq_azul, 'FontWeight', 'bold');
    ylabel(ax1, 'Tiempo (s)');
    set(ax1, 'XTick', x_vec, 'XTickLabel', etiquetas_activas, 'XTickLabelRotation', 12, 'GridColor', [0.6 0.6 0.6]);
    grid(ax1, 'on'); legend(ax1, {'Búsqueda Inicial', 'Optimización RRT*'}, 'Location', 'northwest');

    % --- SUBPLOT 2: COSTO DE RUTA FINAL ---
    ax2 = subplot(2, 2, 2, 'Parent', fig_dash);
    b2 = bar(ax2, x_vec, [m_act.costo]', 'FaceColor', c_rojo_acc, 'EdgeColor', c_uaq_azul, 'LineWidth', 1);
    title(ax2, 'II. Costo Matemático / Distancia de Ruta (Metros)', 'Color', c_uaq_azul, 'FontWeight', 'bold');
    ylabel(ax2, 'Distancia Euclidiana');
    set(ax2, 'XTick', x_vec, 'XTickLabel', etiquetas_activas, 'XTickLabelRotation', 12, 'GridColor', [0.6 0.6 0.6]);
    grid(ax2, 'on');
    % Añadir etiquetas numéricas encima de cada barra
    for b = 1:num_barras
        text(ax2, x_vec(b), m_act(b).costo * 1.03, sprintf('%.1f', m_act(b).costo), ...
             'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', c_uaq_azul);
    end

    % --- SUBPLOT 3: EFICIENCIA DE MUESTREO ---
    ax3 = subplot(2, 2, 3, 'Parent', fig_dash);
    b3 = bar(ax3, x_vec, [m_act.eficiencia]', 'FaceColor', c_uaq_oro, 'EdgeColor', c_uaq_azul, 'LineWidth', 1);
    title(ax3, 'III. Eficiencia de Muestreo Espacial (%)', 'Color', c_uaq_azul, 'FontWeight', 'bold');
    ylabel(ax3, 'Éxito sin Colisión (%)');
    set(ax3, 'XTick', x_vec, 'XTickLabel', etiquetas_activas, 'XTickLabelRotation', 12, 'YLim', [0 105], 'GridColor', [0.6 0.6 0.6]);
    grid(ax3, 'on');
    for b = 1:num_barras
        text(ax3, x_vec(b), min(100, m_act(b).eficiencia + 4), sprintf('%.1f%%', m_act(b).eficiencia), ...
             'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', c_uaq_azul);
    end

    % --- SUBPLOT 4: COMPRESIÓN DE NODOS ---
    ax4 = subplot(2, 2, 4, 'Parent', fig_dash);
    matriz_nodos = [[m_act.n_arboles]', [m_act.n_camino]'];
    b4 = bar(ax4, x_vec, matriz_nodos, 'grouped');
    if length(b4) >= 1, set(b4(1), 'FaceColor', c_gris_bar); end
    if length(b4) >= 2, set(b4(2), 'FaceColor', c_uaq_azul); end
    title(ax4, 'IV. Carga Computacional vs. Síntesis de Ruta', 'Color', c_uaq_azul, 'FontWeight', 'bold');
    ylabel(ax4, 'Cantidad de Nodos');
    set(ax4, 'XTick', x_vec, 'XTickLabel', etiquetas_activas, 'XTickLabelRotation', 12, 'GridColor', [0.6 0.6 0.6]);
    grid(ax4, 'on'); legend(ax4, {'Nodos Generados en Árbol', 'Vértices en Ruta Final'}, 'Location', 'northeast');

    % --- FUNCIÓN AUXILIAR DE EXTRACCIÓN REGEX ---
    function val = extraer_numero(txt, patron)
        tok = regexp(txt, patron, 'tokens', 'once');
        if ~isempty(tok)
            val = str2double(tok{end});
            if isnan(val), val = 0; end
        else
            val = 0;
        end
    end
end
