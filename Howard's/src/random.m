function random_points = random(center1, radius1, center2, radius2, numPoints)
    % random 在两个球体的交集中随机生成numPoints个点
    % center1, center2: 球体的中心 (1x3 向量)
    % radius1, radius2: 球体的半径 (标量)
    % numPoints: 要生成的点的数量 (标量)
    % random_points: 生成的点的坐标 (3xnumPoints 矩阵)

    % 初始化点的数组
    random_points = zeros(3, numPoints);

    % 计算两个球体的距离
    d = norm(center1 - center2);

    % 检查两个球体是否相交
    if d > radius1 + radius2
        error('两个球体不相交');
    end

    % 在交集区域内生成随机点
    for i = 1:numPoints
        while true
            % 在第一个球体内随机生成一个点
            p = center1 + radius1 * randn(1, 3);
            % 检查该点是否在第二个球体内
            if norm(p - center2) <= radius2
                random_points(:, i) = p';
                break;
            end
        end
    end
end
