function hay_colision = Verificar_Colision_Segmento_V20(node1, node2, obstaculos)
    for i = 1:size(obstaculos, 1)
        vertices = obstaculos{i, 1};
        num_vertices = size(vertices, 1);

        obs_xmin = min(vertices(:,1)); obs_xmax = max(vertices(:,1));
        obs_ymin = min(vertices(:,2)); obs_ymax = max(vertices(:,2));
        seg_xmin = min(node1(1), node2(1)); seg_xmax = max(node1(1), node2(1));
        seg_ymin = min(node1(2), node2(2)); seg_ymax = max(node1(2), node2(2));

        if seg_xmin > obs_xmax || seg_xmax < obs_xmin || seg_ymin > obs_ymax || seg_ymax < obs_ymin
            continue;
        end

        for j = 1:num_vertices
            v1 = vertices(j, :);
            if j < num_vertices
                v2 = vertices(j+1, :);
            else
                v2 = vertices(1, :);
            end

            if Segments_Intersect(node1, node2, v1, v2)
                hay_colision = true;
                return;
            end
        end
    end
    hay_colision = false;
end

function intersecta = Segments_Intersect(A, B, C, D)
    function o = orientation(p, q, r)
        val = (q(2) - p(2)) * (r(1) - q(1)) - (q(1) - p(1)) * (r(2) - q(2));
        if abs(val) < 1e-9, o = 0; return; end
        if val > 0, o = 1; else o = 2; end
    end

    o1 = orientation(A, B, C); o2 = orientation(A, B, D);
    o3 = orientation(C, D, A); o4 = orientation(C, D, B);

    if (o1 ~= o2) && (o3 ~= o4); intersecta = true; return; end

    function on = onSegment(p, q, r)
        on = r(1) <= max(p(1), q(1)) && r(1) >= min(p(1), q(1)) && ...
             r(2) <= max(p(2), q(2)) && r(2) >= min(p(2), q(2));
    end

    if (o1 == 0 && onSegment(A, B, C)), intersecta = true; return; end
    if (o2 == 0 && onSegment(A, B, D)), intersecta = true; return; end
    if (o3 == 0 && onSegment(C, D, A)), intersecta = true; return; end
    if (o4 == 0 && onSegment(C, D, B)), intersecta = true; return; end

    intersecta = false;
end
