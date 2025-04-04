function f_2linker()
    % 初始化参数
    L1 = 160; % 第一连杆长度
    L2 = sqrt(73^2+164^2); % 第二连杆长度
    theta3 = atan(73/164); % 修正1：使用正确的反正切函数
    
    % 创建图形界面
    fig = figure('Name', '二连杆逆运动学仿真', 'NumberTitle', 'off');
    
    % 创建坐标输入滑杆
    x_slider = uicontrol('Style', 'slider',...
        'Min', -500, 'Max', 500, 'Value', 0,...
        'Position', [20 80 120 20],...
        'Tooltip', 'X坐标');
    y_slider = uicontrol('Style', 'slider',...
        'Min', -500, 'Max', 500, 'Value', 0,...
        'Position', [20 40 120 20],...
        'Tooltip', 'Y坐标');
    
    % 标签
    uicontrol('Style', 'text', 'Position', [20 100 120 20], 'String', 'X坐标');
    uicontrol('Style', 'text', 'Position', [20 60 120 20], 'String', 'Y坐标');
    
    % 创建坐标轴
    ax = axes('Parent', fig, 'Position', [0.3 0.2 0.6 0.7]);
    hold(ax, 'on');
    axis(ax, 'equal');
    grid(ax, 'on');
    xlim(ax, [-600 600]);
    ylim(ax, [-600 600]);
    
    % 初始化连杆图形
    link1 = plot(ax, [0 0], [0 0], 'b', 'LineWidth', 3);
    link2 = plot(ax, [0 0], [0 0], 'r', 'LineWidth', 3);
    end_effector = plot(ax, 0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    
    % 实时更新监听
    addlistener(x_slider, 'Value', 'PostSet', @(src,evt) updateLinks());
    addlistener(y_slider, 'Value', 'PostSet', @(src,evt) updateLinks());
    
    % 更新连杆函数
    function updateLinks()
        % 获取末端坐标
        x = get(x_slider, 'Value');
        y = get(y_slider, 'Value');
        
        % 执行逆解算
        [theta1, theta2] = inverseKinematics(x, y, L1, L2, theta3); % 修正2：添加theta3参数
        
        % 正向运动学验证
        x1 = L1 * cos(pi - theta1);
        y1 = L1 * sin(pi - theta1);
        x2 = x1 + L2 * cos(theta2 - theta1 + theta3);
        y2 = y1 + L2 * sin(theta2 - theta1 + theta3);
        
        % 更新图形
        set(link1, 'XData', [0 x1], 'YData', [0 y1]);
        set(link2, 'XData', [x1 x2], 'YData', [y1 y2]);
        set(end_effector, 'XData', x2, 'YData', y2);
        title(ax, sprintf('θ1: %.2f°, θ2: %.2f°', rad2deg(theta1), rad2deg(theta2)));
        drawnow;
    end

    % 修正后的逆解算函数
    function [theta1, theta2] = inverseKinematics(x, y, L1, L2, theta3) % 修正3：添加theta3参数
        R = sqrt(x^2 + y^2); % 修正4：使用实际坐标计算R
        
        % 检查可达性
        if R > (L1 + L2) || R < abs(L1 - L2)
            error('目标点不可达！当前R=%.2f，有效范围[%.2f, %.2f]',...
                R, abs(L1-L2), L1+L2);
        end
        
        % 修正数学模型
        D = (R^2 - L1^2 - L2^2) / (2*L1*L2); % 修正5：正确的分母括号
        theta2 = acos(D);
        
        alpha = atan2(y, x);
        beta = acos((L1^2 + R^2 - L2^2)/(2*L1*R)); % 修正6：分母括号
        theta1 = alpha - beta;
        
        % 角度规范化
        theta1 = wrapToPi(theta1);
        theta2 = wrapToPi(theta2 - theta3); % 修正7：考虑theta3补偿
    end

    % 初始更新
    updateLinks();
end