% Algoritmo RRT
% Martin Luna Rogel
%
% Descripción:
%   Algoritmo RRT (Rapidly-exploring Random Tree). 
%   El algoritmo genera un árbol de nodos aleatorios que crece desde un punto
%   inicial hasta alcanzar un punto objetivo dentro de un área
%   definida por dos limites.
%
% Entradas:
%   - Ninguna.
%
% Salidas:
%   - Visualización grafica:
%       • Punto inicial (verde).
%       • Punto final (rojo).
%       • Expansión del árbol (líneas azules).
%       • Camino final encontrado (línea roja ).


clc; clear; close all;

% Parámetros del espacio y del algoritmo
x_lim = [0, 100];   
y_lim = [0, 100];  
inicio = [10, 10];  
objetivo = [90, 90];   
tolerancia = 5; 
max_iter = 1000;    
step_size = 3;     

% Inicialización de estructuras
nodes = inicio;      
parent = 0;          
parent(1) = 0;     

% Dibujar el espacio
figure;
hold on; grid on;
xlim(x_lim); ylim(y_lim);
plot(inicio(1), inicio(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); 
plot(objetivo(1), objetivo(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');  
title('Algoritmo RRT');
xlabel('X'); ylabel('Y');

% Algoritmo RRT
for i = 1:max_iter
    % 1. Generar un punto aleatorio en el espacio
    rand_point = [rand*(x_lim(2)-x_lim(1)), ...
                  rand*(y_lim(2)-y_lim(1))];
    
    % 2. Buscar el nodo más cercano
    dif = nodes - rand_point;                  % diferencias con todos los nodos
    distancias = sqrt(sum(dif.^2, 2));         % norma euclídea de cada diferencia
    [~, idx] = min(distancias);                % índice del nodo más cercano
    nearest_node = nodes(idx, :);

    % 3. Avanzar desde el nodo más cercano hacia el punto aleatorio
    direction = (rand_point - nearest_node);  
    direction = direction / norm(direction); 
    new_node = nearest_node + step_size * direction;
    
    % 4. Agregar el nuevo nodo al árbol
    nodes = [nodes; new_node];
    parent(end+1) = idx;
    
    % 5. Dibujar la conexión
    plot([nearest_node(1), new_node(1)], [nearest_node(2), new_node(2)], 'b');
    drawnow;
    
    % 6. Verificar si se alcanzó el objetivo
    if norm(new_node - objetivo) < tolerancia
        disp('Objetivo alcanzado!');
        
        % Reconstruir la trayectoria desde el nodo final al inicio
        path = new_node;
        p = size(nodes,1);
        while parent(p) ~= 0
            p = parent(p);
            path = [nodes(p,:); path];
        end
  
        plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2);
        break;
    end
end