function nmpc_track_trajectory_with_ode45()
    % Thông số hệ thống
    N = 10; % Số bước thời gian của horizon
    T = 0.2; % Thời gian mỗi bước
    x0 = [0; 0; 0; 0]; % Trạng thái ban đầu
    
    % Điều kiện biên điều khiển
    lb = repmat([-1; -0.5], N, 1);
    ub = repmat([1; 0.5], N, 1);
    
    % Khởi tạo điều khiển ban đầu
    u0 = zeros(2*N, 1);
    
    % Hàm chi phí
    cost_function = @(u) compute_cost_with_ode45(u, x0, N, T);
    
    % Ràng buộc động học
    nonlcon = @(u) dynamic_constraints_with_ode45(u, x0, N, T);
    
    % Giải bài toán tối ưu
    options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');
    u_opt = fmincon(cost_function, u0, [], [], [], [], lb, ub, nonlcon, options);
    
    % Hiển thị kết quả điều khiển tối ưu
    disp('Điều khiển tối ưu:');
    disp(reshape(u_opt, 2, N));
    
    % Mô phỏng hệ thống với điều khiển tối ưu
    simulate_system_with_ode45(x0, u_opt, N, T);
end

function cost = compute_cost_with_ode45(u, x0, N, T)
    % Khởi tạo chi phí
    cost = 0;
    x = x0;
    
    for k = 1:N
        uk = u((2*k-1):(2*k));
        
        % Giải ODE để tính toán trạng thái kế tiếp
        [~, x_next] = ode45(@(t, x) car_dynamics_ode(t, x, uk), [0 T], x);
        x_next = x_next(end, :)';
        
        % Tính toán quỹ đạo mong muốn tại bước k
        xt = desired_trajectory(k, T);
        
        cost = cost + norm(x_next - xt)^2 + norm(uk)^2;
        x = x_next;
    end
end

function [c, ceq] = dynamic_constraints_with_ode45(u, x0, N, T)
    % Khởi tạo trạng thái ban đầu
    x = x0;
    ceq = []; % Vector ràng buộc đẳng thức

    % Duyệt qua mỗi bước thời gian
    for k = 1:N
        % Trích xuất điều khiển tại bước k
        uk = u((2*k-1):(2*k));
        
        % Giải ODE để tính toán trạng thái kế tiếp
        [~, x_next] = ode45(@(t, x) car_dynamics_ode(t, x, uk), [0 T], x);
        x_next = x_next(end, :)';
        
        % Thêm ràng buộc đẳng thức: trạng thái kế tiếp phải bằng trạng thái hiện tại
        ceq = [ceq; x_next - x];
        
        % Cập nhật trạng thái hiện tại
        x = x_next;
    end

    % Không có ràng buộc bất đẳng thức
    c = [];
end

function dxdt = car_dynamics_ode(t, x, u)
    % Động học của xe tự hành
    px = x(1);
    py = x(2);
    theta = x(3);
    v = x(4);
    a = u(1);
    delta = u(2);
    
    beta = atan(0.5 * tan(delta)); % Hệ số trượt
    dpxdt = v * cos(theta + beta);
    dpydt = v * sin(theta + beta);
    dthetadt = v * sin(beta) / 0.5;
    dvdt = a;
    
    dxdt = [dpxdt; dpydt; dthetadt; dvdt];
end

function xt = desired_trajectory(k, T)
    % Định nghĩa quỹ đạo mong muốn (đường thẳng từ (0,0) đến (10,10))
    t = k * T;
    xt = [t; t; 0; 0]; % Đơn giản: px = py = t, theta = 0, v = 0
end

function simulate_system_with_ode45(x0, u_opt, N, T)
    % Mô phỏng hệ thống
    x = x0;
    trajectory = x';
    
    for k = 1:N
        uk = u_opt((2*k-1):(2*k));
        [~, x] = ode45(@(t, x) car_dynamics_ode(t, x, uk), [0 T], x);
        x = x(end, :)';
        trajectory = [trajectory; x'];
    end
    
    % Vẽ quỹ đạo
    figure;
    plot(trajectory(:,1), trajectory(:,2), '-o', 'DisplayName', 'Quỹ đạo xe tự hành');
    hold on;
    
    % Vẽ quỹ đạo mong muốn
    desired = arrayfun(@(k) desired_trajectory(k, T), 1:N+1, 'UniformOutput', false);
    desired = cell2mat(desired');
    plot(desired(:,1), desired(:,2), 'r--', 'DisplayName', 'Quỹ đạo mong muốn');
    
    xlabel('px');
    ylabel('py');
    legend show;
    grid on;
    title('Quỹ đạo của xe tự hành và quỹ đạo mong muốn');
end
