close all
clear all
clc
warning off;

%% Create an object for the support functions.
constants=initial_constants();

%% Load the constant values needed in the main file
Ts=constants('Ts');
outputs=constants('outputs');
hz = constants('hz');
inputs=constants('inputs');
trajectory=constants('trajectory');
Q = constants('Q');
R = constants('R');

%% Create the time array
t = 0:Ts:constants('time_length');

%% Import trajectory generation values
[x_dot_ref,y_dot_ref,psi_ref,X_ref,Y_ref]=trajectory_generator(t);
sim_length=length(t);

%% Generate the reference signal array
refSignals=zeros(1,length(X_ref)*outputs);
k=1;
for i = 1:outputs:length(refSignals)
   refSignals(i)=x_dot_ref(k);
   refSignals(i+1)=psi_ref(k);
   refSignals(i+2)=X_ref(k);
   refSignals(i+3)=Y_ref(k);
   k=k+1;
end
clear i k

%% Load the initial states
x_dot=x_dot_ref(1);
y_dot=y_dot_ref(1);
psi=psi_ref(1);
psi_dot=0;
X=X_ref(1);
Y=Y_ref(1);

%% Create state arrays
states=[x_dot,y_dot,psi,psi_dot,X,Y];
statesTotal=zeros(length(t),length(states));
statesTotal(1,:)=states;

%% Accelerations
x_dot_dot=0;
y_dot_dot=0;
psi_dot_dot=0;

accelerations=[x_dot_dot,y_dot_dot,psi_dot_dot];
accelerations_total=zeros(length(t),length(accelerations));

%% Initiate the controller - simulation loops
U1=0;
U2=0;
UTotal=zeros(length(t),2);
UTotal(1,1)=U1;
UTotal(1,2)=U2;

u=zeros(inputs*hz,1);

times = zeros(sim_length-1, 1);
error_x = 0;
error_y = 0;
error_psi = 0;
error_x_dot = 0;
%% Start with the loop
k=1; % for reading reference signals
for i =1:sim_length-1
    k=k+outputs;
    if k+outputs*hz-1 <= length(refSignals)
        r=refSignals(k:k+outputs*hz-1);
    else
        r=refSignals(k:length(refSignals));
        hz=hz-1;
    end
    
     % Điều kiện biên điều khiển
    lb = repmat([-pi/6; -5], hz, 1);
    ub = repmat([pi/6; 5], hz, 1);
    
    cost_function = @(u) compute_cost(u, r , states, hz, Q, R);
    
    % Ràng buộc động học
    nonlcon = @(u) dynamic_constraints(u, states, hz);
    
    % Giải bài toán tối ưu
    options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');
    tic;
    u_opt = fmincon(cost_function, u, [], [], [], [], lb, ub, nonlcon, options);
    times(i)=toc;
    
    if length(u_opt)==0
        'The solver could not find the solution'
    end
    u=u_opt
    
    U1=u_opt(1);
    U2=u_opt(2);
    
    UTotal(i+1,1)=U1;
    UTotal(i+1,2)=U2;
    
    time_interval=(Ts)/30;
    T = (Ts)*(i-1):time_interval:Ts*(i-1)+(Ts);
    [T,x]=ode45(@(t,x) open_loop_new_states(t,x,[U1,U2]),T,states);
    
    states=x(end,:);
    statesTotal(i+1,:)=states;
    error_x = error_x +abs(states(5)-refSignals(4*i+3));
    error_y = error_y +abs(states(6)-refSignals(4*i+4));
    error_psi = error_psi +abs(states(3)-refSignals(4*i+2));
    error_x_dot = error_x_dot +abs(states(1)-refSignals(4*i+1));
    
    % Accelerations
    x_dot_dot=(x(end,1)-x(end-1,1))/time_interval;
    y_dot_dot=(x(end,2)-x(end-1,2))/time_interval;
    psi_dot_dot=(x(end,4)-x(end-1,4))/time_interval;
    
    accelerations=[x_dot_dot,y_dot_dot,psi_dot_dot];
    accelerations_total(i+1,:)=accelerations;
    
    if mod(i,500)==0
        'Progress (%) '
        i/sim_length*100
    end
end

avg_time = mean(times);
%max = max(times);
fprintf('Thời gian chạy trung bình: %.6f giây\n', avg_time);
fprintf('Thời gian chạy max trong một vòng lặp: %.6f giây\n', max(times));
fprintf('error_x: %.6f \n', error_x/sim_length);
fprintf('error_y: %.6f \n', error_y/sim_length);
fprintf('error_psi: %.6f \n', error_psi/sim_length);
fprintf('error_x_dot: %.6f \n', error_x_dot/sim_length);
%% Plot the trajectory
figure;
plot(X_ref,Y_ref,'--b','LineWidth',2)
hold on
plot(statesTotal(:,5),statesTotal(:,6),'r','LineWidth',1)
grid on;
xlabel('x-position [m]','FontSize',15)
ylabel('y-position [m]','FontSize',15)
%plotObstacles(Obstacles);
legend({'position-ref','position'},'Location','southeast','FontSize',15)

% Plot the inputs
figure;
subplot(2,1,1)
plot(t,UTotal(:,1),'b','LineWidth',2)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('steering wheel angle (delta) [rad]','FontSize',15)
legend({'delta'},'Location','southeast','FontSize',15)

hold on
subplot(2,1,2)
plot(t,UTotal(:,2),'b','LineWidth',2)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('applied acceleration (a) [m/s^2]','FontSize',15)
legend({'a'},'Location','southeast','FontSize',15)

% Plot Psi, X, Y
figure;
subplot(3,1,1)
plot(t,psi_ref,'--b','LineWidth',2)
hold on
plot(t,statesTotal(:,3),'r','LineWidth',1)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('psi-position [rad]','FontSize',15)
legend({'Psi-ref','psi'},'Location','southeast','FontSize',15)

hold on
subplot(3,1,2)
plot(t,X_ref,'--b','LineWidth',2)
hold on
plot(t,statesTotal(:,5),'r','LineWidth',1)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('X-position [m]','FontSize',15)
legend({'X-ref','X'},'Location','southeast','FontSize',15)

hold on
subplot(3,1,3)
plot(t,Y_ref,'--b','LineWidth',2)
hold on
plot(t,statesTotal(:,6),'r','LineWidth',1)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('Y-position [m]','FontSize',15)
legend({'Y-ref','Y'},'Location','southeast','FontSize',15)

% Plot the velocities
figure;
subplot(3,1,1)
plot(t,x_dot_ref,'--b','LineWidth',2)
hold on
plot(t,statesTotal(:,1),'r','LineWidth',1)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('x-dot [m/s]','FontSize',15)
legend({'x-dot-ref','x-dot'},'Location','southeast','FontSize',15)

hold on
subplot(3,1,2)
%plot(t,y_dot_ref,'--b','LineWidth',2)
hold on
plot(t,statesTotal(:,2),'r','LineWidth',1)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('y-dot [m/s]','FontSize',15)
legend({'y-dot'},'Location','southeast','FontSize',15)

hold on
subplot(3,1,3)
plot(t,statesTotal(:,4),'r','LineWidth',1)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('psi-dot [rad/s]','FontSize',15)
legend({'psi-dot'},'Location','southeast','FontSize',15)

% Plot the accelerations
figure;
subplot(3,1,1)
plot(t,accelerations_total(:,1),'b','LineWidth',2)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('x-dot-dot [m/s^2]','FontSize',15)
legend({'x-dot-dot'},'Location','southeast','FontSize',15)

hold on
subplot(3,1,2)
plot(t,accelerations_total(:,2),'b','LineWidth',2)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('y-dot-dot [m/s^2]','FontSize',15)
legend({'y-dot-dot'},'Location','southeast','FontSize',15)

hold on
subplot(3,1,3)
plot(t,accelerations_total(:,3),'b','LineWidth',2)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('psi-dot-dot [rad/s^2]','FontSize',15)
legend({'psi-dot-dot'},'Location','southeast','FontSize',15)

%% cost function
function cost = compute_cost(u, r, x0, N, Q, R)
    % Khởi tạo chi phí
    cost = 0;
    x = x0;
    
    for k = 1:N
        uk = u((2*k-1):(2*k));
        
        % Giải ODE để tính toán trạng thái kế tiếp
        x_next = vehicle_dynamics(x, uk);
        x_next = x_next';
        %x_next = x_next(end, :);
        x_dot = x_next(1);
        psi = x_next(3);
        X_next = x_next(5);
        Y_next = x_next(6);
        state = [x_dot, psi, X_next, Y_next];
        
        % Tính toán quỹ đạo mong muốn tại bước k
        xt = r(4*k-3:4*k);
        %xt = desired_trajectory(k, T);
        
        cost = cost + sum(Q.*norm(state-xt)^2) + sum(R.*norm(uk)^2);
        x = x_next;
    end
end
 
%% dynamic constraints
function [c, ceq] = dynamic_constraints(u, x0, N)
    % Khởi tạo trạng thái ban đầu
    x = x0;
    
    c = []; % Vector ràng buộc bất đẳng thức

    % Duyệt qua mỗi bước thời gian
    for k = 1:N
        % Trích xuất điều khiển tại bước k
        uk = u((2*k-1):(2*k));
        
        % Giải ODE để tính toán trạng thái kế tiếp
        x_next = vehicle_dynamics(x, uk);
        x_next = x_next';
        %x_next = x_trajectory(end, :);
        x_dot = x_next(1);
        y_dot = x_next(2);
        % Thêm ràng buộc đẳng thức: trạng thái kế tiếp phải phù hợp với động học
        %ceq = [ceq; x_next - x];
        
        % Thêm ràng buộc bất đẳng thức: vận tốc phải trong khoảng [1, 20]
        c = [c; 1 - x_dot; x_dot - 30; -3-y_dot; y_dot-3];
        
        % Cập nhật trạng thái hiện tại
        x = x_next';
    end
    ceq = []; % Vector ràng buộc đẳng thức
end
