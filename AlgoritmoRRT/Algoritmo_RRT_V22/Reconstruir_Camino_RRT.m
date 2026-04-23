% Reconstruir_Camino_RRT.m
% =========================================================================
% === FUNCION PARA OBTENER COORDENADAS DESDE UN NODO HASTA LA RAIZ ===
%
% Descripción: Sigue la cadena de padres desde el nodo de unión hasta
% el nodo inicial (raíz) y devuelve los índices y coordenadas.
% =========================================================================

function [path_coords, path_indices] = Reconstruir_Camino_RRT(idx_union, nodes, parent)
    path_indices = [];
    p = idx_union; % Comenzamos desde el punto donde se conectaron los árboles.

    % Se itera hacia atrás mientras el nodo tenga un padre (la raíz tiene parent = 0)
    while parent(p) ~= 0
        path_indices = [p; path_indices]; % Agrega el índice al inicio de la lista.
        p = parent(p); % Salta al padre.
    end

    path_indices = [1; path_indices]; % Añade el índice 1 (la raíz del árbol).
    path_coords = nodes(path_indices, :); % Extrae las coordenadas de esos índices.
end
