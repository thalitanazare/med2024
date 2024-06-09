clear all
clc
close all
%% Global variable to store optimization results
global optimization_results Va_global Klqr cost_history;
optimization_results = [];
cost_history=[];
%% Initial motor parameters
%[1] Resistance - R
%[2] Indutance - L
%[3] Torque Constant - Km
%[4] Back-EMF Contante - Kb
%[5] Friction Coefficient - Kf
%[6] Inertia - J
%[7] LQR parameter - q11
%[8] LQR parameter - q22
%[9] LQR parameter - r
params_initial =[0.5, 0.01, 1.23, 1.23,0.02,0.05,0.5,10,0.5];
%% Simulation Time and Disturbances Intervaz
t = 0:0.01:25; %simulation time
Td=zeros(size(t));
interval = (t > 5 & t < 15);
%% Disturbance 01 - Step Overload
% Td = 45 * interval;
%% Disturbance 02 - Ramp Overload
interval1 = t > 8 & t <= 12;
interval2 = t >= 12 & t < 16;
Td(interval1) = linspace(0, 100, sum(interval1));
Td(interval2) = linspace(100, 0, sum(interval2));
%% Disturbance 03 - Impulse Overload
%Impulse in the midle of each interval
% midle_int = [15, 30]; % Midle Value
% for i = 1:length(midle_int)
%     [~, idx] = min(abs(t - midle_int(i)));
%     Td(idx) = 50; % Impulse Value (Amplitude)
% end

%% NO OPTIMIZATION
% Unpack the parameters
R = params_initial(1);
L = params_initial(2);
Km = params_initial(3);
Kb = params_initial(4);
Kf = params_initial(5);
J = params_initial(6);
% for the controller
q1= params_initial(7);
q2= params_initial(8);
r= params_initial(9);

%veja o diagrama de blocos
h1 = tf(Km,[L R]);            % armature
h2 = tf(1,[J Kf]);            % eqn of motion

dcm = ss(h2) * [h1 , 1];      % w = h2 * (h1*Va + Td)
dcm = feedback(dcm,Kb,1,1);   % close back emf loop

%dcm=tf(Km,[J*L R*J+Kf*L Km^2+R]);

%dcm=ss(dcm);
dc_aug = [1 ; tf(1,[1 0])] * dcm(1); % add output w/s to DC motor model

K_lqr = lqry(dc_aug,[q1 0;0 q2],r);
P = augstate(dcm);                     % inputs:Va,Td  outputs:w,x
C = K_lqr * append(tf(1,[1 0]),1,1);   % compensator including 1/s
OL = P * append(C,1);                  % open loop

CL = feedback(OL,eye(3),1:3,1:3);      % close feedback loops
sys = CL(1,[1 4]);                  % extract transfer (w_ref,Td)->w
u = [ones(size(t))*100; Td];

[y, T, X] = lsim(sys, u', t);

i = X(:,1);
w = X(:,2);
integral_w = X(:,3);

Va_initial= K_lqr(1) * i + K_lqr(2) * w + K_lqr(3) * integral_w;

power1 = Va_initial .* i;
energy1 = trapz(t, power1); % Numerical integration

Va_inital_max=max(Va_initial);
% figure
% plot(Va)
% 
% figure
% h = lsimplot(sys,u,t);
%% Optimization settings
clear R L Km Kb Kf J q1 q2 r h1 h2 dcm dc_aug K_lqr P C OL CL i w integral_w Va_initial
param_bounds = params_initial(1:9) * 0.2;
lb = params_initial - param_bounds;
ub = params_initial + param_bounds;
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp','OutputFcn', @outfun);
options.OptimalityTolerance = 1e-6; % exemplo: 1e-6
% Run the optimization
[opt_params, ~] = fmincon(@(p) optimize_motor(p,Td,t), params_initial, [], [], [], [], lb, ub, [], options);

%% Compare response and ploting results
u = [ones(size(t))*100; Td];  % w_ref=100 and Td
% Response with initial parameters
y_initial = lsim(get_motor_response(params_initial), u, t);
% Response with optimized parameters
y_optimized = lsim(get_motor_response(opt_params), u, t);


%% Creating a table to store the results
% Define parameter names
param_names = {'R (Ohms)', 'L (Henrys)', 'Km (Torque Constant)', 'Kb (Back EMF Constant)', 'Kf (Friction Constant)', 'J (Inertia)', 'q1','q2', 'r'};

% Create tables for initial and optimized parameters
initial_params_table = array2table(params_initial, 'VariableNames', param_names);
optimized_params_table = array2table(opt_params, 'VariableNames', param_names);

% Calculate metrics for initial and optimized parameters
[initial_cost,  initial_overshoot, initial_energy, cl_lqr_initial,Va_initial] = optimize_motor(params_initial, Td, t);
[optimized_cost,  optimized_overshoot, optimized_energy, cl_lqr_optimized,Va_final] = optimize_motor(opt_params, Td, t);
Va_initial_max=max(Va_initial);
Va_final_max=max(Va_final);
% Add metrics to tables
initial_params_table = [initial_params_table, array2table([initial_cost, initial_overshoot, initial_energy,Va_initial_max], 'VariableNames', {'Cost', 'Overshoot', 'Energy','Va_max'})];
optimized_params_table = [optimized_params_table, array2table([optimized_cost,  optimized_overshoot, optimized_energy,Va_final_max], 'VariableNames', {'Cost',  'Overshoot', 'Energy','Va_max'})];

% Combine tables for comparison
comparison_table = [initial_params_table; optimized_params_table];

% View the comparison table
disp('Comparison of Motor Parameters: Initial vs Optimized');
disp(comparison_table);

%% Saving data
% save('motor_performance_data_D02_C03.mat', 'Td', 'y_initial', 'y_optimized');
% save('Va_D02_C03.mat', 'Va_initial', 'Va_final')
save('cost_history_D02_C03.mat', 'cost_history');
%% Plotting
% figure;
% plot(t, y_initial, 'Color', [0, 0.45, 0.73], 'LineWidth', 2.5);
% hold on;
% plot(t, y_optimized, 'Color', [0.7, 0.11, 0.11], 'LineWidth', 2);
% hold on;
% plot(t, Td, 'k--', 'LineWidth', 2); %
% set(gca, 'FontSize', 20, 'FontName', 'Times');
% xlabel('s', 'Interpreter', 'latex', 'FontSize', 20);
% ylabel('$\omega$', 'Interpreter', 'latex', 'FontSize', 20);
% title('Motor Performance: Initial vs Optimised Parameters', 'Interpreter', 'latex', 'FontSize', 20);
% %legend('Initial Parameters', 'Optimised Parameters', 'Interpreter', 'latex', 'Location', 'best');
% grid on;
% 
% figure
% plot(Va_global)

%% Optimisation Function
function [cost, overshoot,energy,...
    cl_lqr,Va] = optimize_motor(params,Td,t)
%% Unpack the parameters
R = params(1);
L = params(2);
Km = params(3);
Kb = params(4);
Kf = params(5);
J = params(6);
% for the controller
q1= params(7);
q2= params(8);
r= params(9);
%% State-space model of the DC motor
h1 = tf(Km,[L R]);            % armature
h2 = tf(1,[J Kf]);            % eqn of motion
dcm = ss(h2) * [h1 , 1];      % w = h2 * (h1*Va + Td)
dcm = feedback(dcm,Kb,1,1);   % close back emf loop
%% LQR DC Motor Control Design
dc_aug = [1 ; tf(1,[1 0])] * dcm(1); % add output w/s to DC motor model
Klqr = lqry(dc_aug, [q1 0; 0 q2], r);
P = augstate(dcm);                     % inputs:Va,Td  outputs:w,x
C = Klqr * append(tf(1,[1 0]),1,1);   % compensator including 1/s
OL = P * append(C,1);                  % open loop
CL = feedback(OL,eye(3),1:3,1:3);      % close feedback loops
cl_lqr = CL(1,[1 4]);
% dc_aug = [tf(1,[1 0])] * dcm(1);
% Klqr=lqr(dc_aug,Q,r);
% P = augstate(dcm);
% OL = P * Klqr;
% CL = feedback(OL, eye(3), 1:3, 1:3);
% cl_lqr = CL(1, 1:3);
u = [ones(size(t))*100; Td];
[y, T, X] = lsim(cl_lqr, u', t);
% X(:,1) is the armature current 'i' and X(:,2) is the angular velocity 'w'
i = X(:,1);
w = X(:,2);
integral_w = X(:,3);

% Now apply the LQR earnings.
% Assuming that K_lqr = [K1 K2 K3], where K2 is applied
% to the integral of 'w'.
Va= Klqr(1) * i + Klqr(2) * w  + Klqr(3)*integral_w;
% Calculate the energy (integral of power)
%power = (Va.^2)/R; % Element-wise multiplication
power = Va .* i;
energy = trapz(t, power)/1000000; % Numerical integration

%% Calculate performance metrics
info = stepinfo(y, t);
overshoot = info.Overshoot/10;

%% Cost function with normalized metrics
cost = 0.1*overshoot + 0.9*energy;
%% Save the results
global optimization_results Va_global K_lqr;
optimization_results = [optimization_results; params, cost, ...
    overshoot, ...
    energy];
Va_global=Va;
K_lqr=Klqr;
end

function stop = outfun(x, optimValues, state)
    stop = false; % Não parar o algoritmo
    global cost_history
    switch state
        case 'iter'
            % Concatena os resultados atuais com os anteriores
            cost_history = [cost_history; optimValues.fval, optimValues.funccount];
    end
end