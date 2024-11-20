close all
clear all
clc
warning off;
%load('D:\Downloads\Lab_thầy_Khoa\Đồ án\Code\MATLAB version - All files\autonomous_car2_360tracking\TH4_lin.mat')
%load('D:\Downloads\Lab_thầy_Khoa\Đồ án\Code\MATLAB version - All files\NMPC\final_non3.mat')
%load('D:\Downloads\Lab_thầy_Khoa\Đồ án\Code\MATLAB version - All files\LPV MPC\final3_30.mat')
load('D:\Downloads\Lab_thầy_Khoa\Đồ án\Code\MATLAB version - All files\NMPC - Copy\final_non3_30.mat')


%% Create an object for the support functions.
constants=initial_constants();

%% Load the constant values needed in the main file
Ts=constants('Ts');
outputs=constants('outputs');
hz = constants('hz');
inputs=constants('inputs');
trajectory=constants('trajectory');

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


%%
figure;
plot(X_ref,Y_ref,'--b','LineWidth',2)
hold on
plot(statesTotal_final_non3(:,5),statesTotal_final_non3(:,6),'r','LineWidth',1)
grid on;
xlabel('x_G [m]','FontSize',15)
ylabel('y_G [m]','FontSize',15)
%plotObstacles(Obstacles);
legend({'position-ref','position-LPV MPC'},'Location','southeast','FontSize',15)

% Plot the inputs
figure;
subplot(2,1,1)
plot(t,UTotal_final_non3(:,1),'b','LineWidth',2)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('steering wheel angle (\delta_f) [rad]','FontSize',15)
legend({'\delta_f'},'Location','southeast','FontSize',15)

hold on
subplot(2,1,2)
plot(t,UTotal_final_non3(:,2),'b','LineWidth',2)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('applied acceleration (a) [m/s^2]','FontSize',15)
legend({'a'},'Location','southeast','FontSize',15)

% Plot Psi, X, Y
figure;
subplot(3,1,1)
plot(t,psi_ref,'--b','LineWidth',2)
hold on
plot(t,statesTotal_final_non3(:,3),'r','LineWidth',1)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('phi [rad]','FontSize',15)
legend({'phi-ref','phi-LPV MPC'},'Location','southeast','FontSize',15)

hold on
subplot(3,1,2)
plot(t,X_ref,'--b','LineWidth',2)
hold on
plot(t,statesTotal_final_non3(:,5),'r','LineWidth',1)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('x_G [m]','FontSize',15)
legend({'x_G-ref','x_G-LPV MPC'},'Location','southeast','FontSize',15)

hold on
subplot(3,1,3)
plot(t,Y_ref,'--b','LineWidth',2)
hold on
plot(t,statesTotal_final_non3(:,6),'r','LineWidth',1)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('y_G [m]','FontSize',15)
legend({'y_G-ref','y_G-LPV MPC'},'Location','southeast','FontSize',15)

% Plot the velocities
figure;
subplot(3,1,1)
plot(t,x_dot_ref,'--b','LineWidth',2)
hold on
plot(t,statesTotal_final_non3(:,1),'r','LineWidth',1)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('dx [m/s]','FontSize',15)
legend({'dx-ref','dx-LPV MPC'},'Location','southeast','FontSize',15)

hold on
subplot(3,1,2)
plot(t,y_dot_ref,'--b','LineWidth',2)
hold on
plot(t,statesTotal_final_non3(:,2),'r','LineWidth',1)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('dy [m/s]','FontSize',15)
legend({'dy-ref','dy-LPV MPC'},'Location','southeast','FontSize',15)

hold on
subplot(3,1,3)
plot(t,statesTotal_final_non3(:,4),'r','LineWidth',1)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('dphi [rad/s]','FontSize',15)
legend({'dphi-LPV MPC'},'Location','southeast','FontSize',15)

% Plot the accelerations
figure;
subplot(3,1,1)
plot(t,accelerations_total_final_non3(:,1),'b','LineWidth',2)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('ddx[m/s^2]','FontSize',15)
legend({'ddx-LPV MPC'},'Location','southeast','FontSize',15)

hold on
subplot(3,1,2)
plot(t,accelerations_total_final_non3(:,2),'b','LineWidth',2)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('ddy [m/s^2]','FontSize',15)
legend({'ddy-LPV MPC'},'Location','southeast','FontSize',15)

hold on
subplot(3,1,3)
plot(t,accelerations_total_final_non3(:,3),'b','LineWidth',2)
grid on
xlabel('t-time [s]','FontSize',15)
ylabel('ddphi [rad/s^2]','FontSize',15)
legend({'ddphi-LPV MPC'},'Location','southeast','FontSize',15)
