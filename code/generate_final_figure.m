clear all 
clc
close all

t = 0:0.01:25; %simulation time
% %% Disturbance 01
% D01_C01=load('motor_performance_data_D01_C01.mat');
% D01_C02=load('motor_performance_data_D01_C02.mat');
% D01_C03=load('motor_performance_data_D01_C03.mat');
% 
% D01_Td=D01_C01.Td;
% D01_y_initial=D01_C01.y_initial;
% % Motor Perfomance 
% D01_C01_y_op=D01_C01.y_optimized;
% D01_C02_y_op=D01_C02.y_optimized;
% D01_C03_y_op=D01_C03.y_optimized;
% 
% 
% % Plotting
% figure;
% plot(t, D01_y_initial, 'Color', [0, 0.45, 0.73], 'LineWidth', 3.3); % Darker cobalt blue line for y_initial
% hold on;
% plot(t, D01_C01_y_op, 'Color', [1, 0.55, 0.1], 'LineWidth', 3.5); % Marsala red line for y_optimized
% hold on;
% plot(t, D01_C02_y_op, 'Color', [0.3, 0.7, 0.2], 'LineWidth', 3); % Marsala red line for y_optimized
% hold on;
% plot(t, D01_C03_y_op, 'Color', [0.7, 0.11, 0.11], 'LineWidth', 3.5); % Marsala red line for y_optimized
% % Setting font size for axis labels and using Latex for text rendering
% set(gca, 'FontSize', 20, 'FontName', 'Times');
% xlabel('s', 'Interpreter', 'latex', 'FontSize', 20);
% ylabel('$\omega$', 'Interpreter', 'latex', 'FontSize', 20);
% title('Motor Performance: Step Disturbance ', 'Interpreter', 'latex', 'FontSize', 20);
% lgd=legend('Initial', '$\lambda=0.9$,$\phi=0.1$', '$\lambda=0.5$,$\phi=0.5$','$\lambda=0.1$,$\phi=0.9$', 'Interpreter', 'latex', 'Location', 'best');
% % Exemplo: movendo a legenda mais para a direita e um pouco para cima
% lgd.Position = [0.61 0.15 lgd.Position(3) lgd.Position(4)];
% % Setting grid
% grid on;
% % Adicionando um círculo na figura principal para indicar a área de zoom
% % A função rectangle com 'Curvature' [1 1] cria um elipse, para um círculo, assegure que a largura e altura sejam iguais
% % 'Position' = [x y width height]
% rectangle('Position', [3.5, 98, 3, 25], 'Curvature', [1, 1], 'EdgeColor', 'k', 'LineWidth', 2, 'LineStyle', '--');
% % Adicionando uma seta para ligar a área de zoom ao gráfico de zoom
% % 'annotation' tipo 'arrow' com coordenadas normalizadas da figura [x_inicial y_inicial x_final y_final]
% % As coordenadas são normalizadas de 0 a 1 em relação à figura inteira
% annotation('arrow', [0.29 0.4], [0.68 0.55], 'LineWidth', 2.5);
% % Adicionando um zoom
% % Definindo a posição e tamanho do eixo de zoom [left bottom width height]
% axes('Position',[0.22 .2 .35 .35]) % Ajuste esses valores conforme necessário
% box on % Adiciona uma caixa ao redor do novo conjunto de eixos
% 
% % Plotando os mesmos dados, mas com limites de eixo focados na região de interesse
% plot(t, D01_y_initial, 'Color', [0, 0.45, 0.73], 'LineWidth', 3);
% hold on;
% plot(t, D01_C01_y_op, 'Color', [1, 0.55, 0.1], 'LineWidth', 3.5);
% plot(t, D01_C02_y_op, 'Color', [0.3, 0.7, 0.2], 'LineWidth', 3);
% plot(t, D01_C03_y_op, 'Color', [0.7, 0.11, 0.11], 'LineWidth', 3.5);
% xlim([4.5 6.5]); % Limites do eixo x
% ylim([98 122]); % Limites do eixo y
% grid on
% % Configurando a aparência dos eixos do zoom
% set(gca, 'FontSize', 18,'FontName', 'Times'); % Tamanho da fonte menor para o zoom
% %xlabel('s', 'Interpreter', 'latex', 'FontSize', 10);
% %ylabel('$\omega$', 'Interpreter', 'latex', 'FontSize', 10);
% %title('Zoom', 'Interpreter', 'latex', 'FontSize', 10);
% 
% 
% savefig('/Users/thalitanazare/Library/CloudStorage/OneDrive-MaynoothUniversity/PhD/My articles/2024/24MED/CCD/R2/code/D01.fig');
% print('/Users/thalitanazare/Library/CloudStorage/OneDrive-MaynoothUniversity/PhD/My articles/2024/24MED/CCD/R2/code/D01', '-depsc');
% 
% 
% 
% 
% %% Disturbance 02
% D02_C01=load('motor_performance_data_D02_C01.mat');
% D02_C02=load('motor_performance_data_D02_C02.mat');
% D02_C03=load('motor_performance_data_D02_C03.mat');
% 
% D02_Td=D02_C01.Td;
% D02_y_initial=D02_C01.y_initial;
% % Motor Perfomance 
% D02_C01_y_op=D02_C01.y_optimized;
% D02_C02_y_op=D02_C02.y_optimized;
% D02_C03_y_op=D02_C03.y_optimized;
% 
% 
% 
% % Plotting
% figure;
% plot(t, D02_y_initial, 'Color', [0, 0.45, 0.73], 'LineWidth', 3); % Darker cobalt blue line for y_initial
% hold on;
% plot(t, D02_C01_y_op, 'Color', [1, 0.55, 0.1], 'LineWidth', 3.5); % Marsala red line for y_optimized
% hold on;
% plot(t, D02_C02_y_op, 'Color', [0.3, 0.7, 0.2], 'LineWidth', 3); % Marsala red line for y_optimized
% hold on;
% plot(t, D02_C03_y_op, 'Color', [0.7, 0.11, 0.11], 'LineWidth', 3.5); % Marsala red line for y_optimized\
% % Setting font size for axis labels and using Latex for text rendering
% set(gca, 'FontSize', 20, 'FontName', 'Times');
% xlabel('s', 'Interpreter', 'latex', 'FontSize', 20);
% ylabel('$\omega$', 'Interpreter', 'latex', 'FontSize', 20);
% title('Motor Performance: Triangular Wave Disturbance', 'Interpreter', 'latex', 'FontSize', 20);
% lgd=legend('Initial', '$\lambda=0.9$,$\phi=0.1$', '$\lambda=0.5$,$\phi=0.5$','$\lambda=0.1$,$\phi=0.9$', 'Interpreter', 'latex', 'Location', 'best');
% lgd.Position = [0.61 0.15 lgd.Position(3) lgd.Position(4)];
% 
% % Setting grid
% grid on;
% % Adicionando um círculo na figura principal para indicar a área de zoom
% % A função rectangle com 'Curvature' [1 1] cria um elipse, para um círculo, assegure que a largura e altura sejam iguais
% % 'Position' = [x y width height]
% rectangle('Position', [7, 92, 10, 15], 'Curvature', [0, 0], 'EdgeColor', 'k', 'LineWidth', 2, 'LineStyle', '--');
% % Adicionando uma seta para ligar a área de zoom ao gráfico de zoom
% % 'annotation' tipo 'arrow' com coordenadas normalizadas da figura [x_inicial y_inicial x_final y_final]
% % As coordenadas são normalizadas de 0 a 1 em relação à figura inteira
% annotation('arrow', [0.58 0.49], [0.76 0.55], 'LineWidth', 2.5);
% % Adicionando um zoom
% % Definindo a posição e tamanho do eixo de zoom [left bottom width height]
% axes('Position',[0.22 .2 .35 .35]) % Ajuste esses valores conforme necessário
% box on % Adiciona uma caixa ao redor do novo conjunto de eixos
% 
% % Plotando os mesmos dados, mas com limites de eixo focados na região de interesse
% plot(t, D02_y_initial, 'Color', [0, 0.45, 0.73], 'LineWidth', 3);
% hold on;
% plot(t, D02_C01_y_op, 'Color', [1, 0.55, 0.1], 'LineWidth', 3.5);
% plot(t, D02_C02_y_op, 'Color', [0.3, 0.7, 0.2], 'LineWidth', 3);
% plot(t, D02_C03_y_op, 'Color', [0.7, 0.11, 0.11], 'LineWidth', 3.5);
% xlim([7 18.5]); % Limites do eixo x
% ylim([96 104]); % Limites do eixo y
% grid on
% % Configurando a aparência dos eixos do zoom
% set(gca, 'FontSize', 18,'FontName', 'Times'); % Tamanho da fonte menor para o zoom
% %xlabel('s', 'Interpreter', 'latex', 'FontSize', 10);
% %ylabel('$\omega$', 'Interpreter', 'latex', 'FontSize', 10);
% %title('Zoom', 'Interpreter', 'latex', 'FontSize', 10);
% savefig('/Users/thalitanazare/Library/CloudStorage/OneDrive-MaynoothUniversity/PhD/My articles/2024/24MED/CCD/R2/code/D02.fig');
% print('/Users/thalitanazare/Library/CloudStorage/OneDrive-MaynoothUniversity/PhD/My articles/2024/24MED/CCD/R2/code/D02', '-depsc');
% 
% %% Va result
% % Disturbance 01
% Va01_C01=load('Va_D01_C01.mat');
% Va01_C02=load('Va_D01_C02.mat');
% Va01_C03=load('Va_D01_C03.mat');
% 
% %D02_Td=D02_C01.Td;
% D01_Va_initial=Va01_C01.Va_initial;
% % Motor Perfomance 
% D01_C01_Va_op=Va01_C01.Va_final;
% D01_C02_Va_op=Va01_C02.Va_final;
% D01_C03_Va_op=Va01_C03.Va_final;
% 
% % Plotting
% figure;
% plot(t, D01_Va_initial, 'Color', [0, 0.45, 0.73], 'LineWidth', 3); % Darker cobalt blue line for y_initial
% hold on;
% plot(t, D01_C01_Va_op, 'Color', [1, 0.55, 0.1], 'LineWidth', 3.5); % Marsala red line for y_optimized
% hold on;
% plot(t, D01_C02_Va_op, 'Color', [0.3, 0.7, 0.2], 'LineWidth', 3); % Marsala red line for y_optimized
% hold on;
% plot(t, D01_C03_Va_op, 'Color', [0.7, 0.11, 0.11], 'LineWidth', 3.5); % Marsala red line for y_optimized\
% % Setting font size for axis labels and using Latex for text rendering
% set(gca, 'FontSize', 20, 'FontName', 'Times');
% xlabel('s', 'Interpreter', 'latex', 'FontSize', 20);
% ylabel('$V_a$', 'Interpreter', 'latex', 'FontSize', 20);
% title('Controlled Voltage: Step Disturbance', 'Interpreter', 'latex', 'FontSize', 20);
% lgd=legend('Initial', '$\lambda=0.9$,$\phi=0.1$', '$\lambda=0.5$,$\phi=0.5$','$\lambda=0.1$,$\phi=0.9$', 'Interpreter', 'latex', 'Location', 'best');
% lgd.Position = [0.61 0.15 lgd.Position(3) lgd.Position(4)];
% grid on
% 
% % savefig('/Users/thalitanazare/Library/CloudStorage/OneDrive-MaynoothUniversity/PhD/My articles/2024/24MED/CCD/R2/code/V01.fig');
% % print('/Users/thalitanazare/Library/CloudStorage/OneDrive-MaynoothUniversity/PhD/My articles/2024/24MED/CCD/R2/code/V01', '-depsc');
% 
% 
% % Disturbance 02
% Va02_C01=load('Va_D02_C01.mat');
% Va02_C02=load('Va_D02_C02.mat');
% Va02_C03=load('Va_D02_C03.mat');
% 
% %D02_Td=D02_C01.Td;
% D02_Va_initial=Va02_C01.Va_initial;
% % Motor Perfomance 
% D02_C01_Va_op=Va02_C01.Va_final;
% D02_C02_Va_op=Va02_C02.Va_final;
% D02_C03_Va_op=Va02_C03.Va_final;
% 
% % Plotting
% figure;
% plot(t, D02_Va_initial, 'Color', [0, 0.45, 0.73], 'LineWidth', 3); % Darker cobalt blue line for y_initial
% hold on;
% plot(t, D02_C01_Va_op, 'Color', [1, 0.55, 0.1], 'LineWidth', 3.5); % Marsala red line for y_optimized
% hold on;
% plot(t, D02_C02_Va_op, 'Color', [0.3, 0.7, 0.2], 'LineWidth', 3); % Marsala red line for y_optimized
% hold on;
% plot(t, D02_C03_Va_op, 'Color', [0.7, 0.11, 0.11], 'LineWidth', 3.5); % Marsala red line for y_optimized\
% % Setting font size for axis labels and using Latex for text rendering
% set(gca, 'FontSize', 20, 'FontName', 'Times');
% xlabel('s', 'Interpreter', 'latex', 'FontSize', 20);
% ylabel('$V_a$', 'Interpreter', 'latex', 'FontSize', 20);
% title('Controlled Voltage: Triangular Wave Disturbance', 'Interpreter', 'latex', 'FontSize', 20);
% lgd=legend('Initial', '$\lambda=0.9$,$\phi=0.1$', '$\lambda=0.5$,$\phi=0.5$','$\lambda=0.1$,$\phi=0.9$', 'Interpreter', 'latex', 'Location', 'best');
% lgd.Position = [0.61 0.15 lgd.Position(3) lgd.Position(4)];
% grid on
% 
% savefig('/Users/thalitanazare/Library/CloudStorage/OneDrive-MaynoothUniversity/PhD/My articles/2024/24MED/CCD/R2/code/V02.fig');
% print('/Users/thalitanazare/Library/CloudStorage/OneDrive-MaynoothUniversity/PhD/My articles/2024/24MED/CCD/R2/code/V02', '-depsc');
% 
% %% Plot just disturbance
% 
% figure
% subplot(2,1,1)
% plot(t, D01_Td, 'Color', [0, 0, 0], 'LineWidth', 3); % Darker cobalt blue line for y_initial
% % Setting font size for axis labels and using Latex for text rendering
% set(gca, 'FontSize', 20, 'FontName', 'Times');
% xlabel('s', 'Interpreter', 'latex', 'FontSize', 20);
% ylabel('$T_d$', 'Interpreter', 'latex', 'FontSize', 20);
% title('Step Disturbance', 'Interpreter', 'latex', 'FontSize', 20);
% grid on
% subplot(2,1,2)
% plot(t, D02_Td, 'Color', [0, 0, 0], 'LineWidth', 3); % Darker cobalt blue line for y_initial
% % Setting font size for axis labels and using Latex for text rendering
% set(gca, 'FontSize', 20, 'FontName', 'Times');
% xlabel('s', 'Interpreter', 'latex', 'FontSize', 20);
% ylabel('$T_d$', 'Interpreter', 'latex', 'FontSize', 20);
% title('Triangular Wave Disturbance', 'Interpreter', 'latex', 'FontSize', 20);
% grid on
% 
% savefig('/Users/thalitanazare/Library/CloudStorage/OneDrive-MaynoothUniversity/PhD/My articles/2024/24MED/CCD/R2/code/Td.fig');
% print('/Users/thalitanazare/Library/CloudStorage/OneDrive-MaynoothUniversity/PhD/My articles/2024/24MED/CCD/R2/code/Td', '-depsc');

%% COST
%cost 01
cost_D01_C01=load('cost_history_D01_C01.mat');
cost_D01_C02=load('cost_history_D01_C02.mat');
cost_D01_C03=load('cost_history_D01_C03.mat');


figure
hold on;
h1=plot(cost_D01_C01.cost_history(:,2), cost_D01_C01.cost_history(:,1), '-*','Color', [1, 0.55, 0.1], 'LineWidth', 3);
h2=plot(cost_D01_C02.cost_history(:,2), cost_D01_C02.cost_history(:,1), '-*','Color', [0.3, 0.7, 0.2], 'LineWidth', 3);
h3=plot(cost_D01_C03.cost_history(:,2), cost_D01_C03.cost_history(:,1), '-*','Color', [0.7, 0.11, 0.11], 'LineWidth', 3);
set(gca, 'FontSize', 20, 'FontName', 'Times');
xlabel('Func-count', 'Interpreter', 'latex', 'FontSize', 20);
ylabel('Cost', 'Interpreter', 'latex', 'FontSize', 20);
title('Cost Evolution - Step Disturbance', 'Interpreter', 'latex', 'FontSize', 20);
lgd=legend('$\lambda=0.9$,$\phi=0.1$', '$\lambda=0.5$,$\phi=0.5$','$\lambda=0.1$,$\phi=0.9$', 'Interpreter', 'latex', 'Location', 'best');
lgd.Position = [0.61 0.75 lgd.Position(3) lgd.Position(4)];
grid on
d1=datatip(h1, 'DataIndex', length(cost_D01_C01.cost_history));
d2=datatip(h2, 'DataIndex', length(cost_D01_C02.cost_history));
d3=datatip(h3, 'DataIndex', length(cost_D01_C03.cost_history));
d1.FontSize = 14;
d2.FontSize = 14;
d3.FontSize = 14;
hold off;

%cost 02
cost_D02_C01=load('cost_history_D02_C01.mat');
cost_D02_C02=load('cost_history_D02_C02.mat');
cost_D02_C03=load('cost_history_D02_C03.mat');


figure
hold on;
h1=plot(cost_D02_C01.cost_history(:,2), cost_D02_C01.cost_history(:,1), '-*','Color', [1, 0.55, 0.1], 'LineWidth', 3);
h2=plot(cost_D02_C02.cost_history(:,2), cost_D02_C02.cost_history(:,1), '-*','Color', [0.3, 0.7, 0.2], 'LineWidth', 3);
h3=plot(cost_D02_C03.cost_history(:,2), cost_D02_C03.cost_history(:,1), '-*','Color', [0.7, 0.11, 0.11], 'LineWidth', 3);
set(gca, 'FontSize', 20, 'FontName', 'Times');
xlabel('Func-count', 'Interpreter', 'latex', 'FontSize', 20);
ylabel('Cost', 'Interpreter', 'latex', 'FontSize', 20);
title('Cost Evolution - Triangular Wave Disturbance', 'Interpreter', 'latex', 'FontSize', 20);
lgd=legend('$\lambda=0.9$,$\phi=0.1$', '$\lambda=0.5$,$\phi=0.5$','$\lambda=0.1$,$\phi=0.9$', 'Interpreter', 'latex', 'Location', 'best');
lgd.Position = [0.61 0.75 lgd.Position(3) lgd.Position(4)];
grid on
d1=datatip(h1, 'DataIndex', length(cost_D02_C01.cost_history));
d2=datatip(h2, 'DataIndex', length(cost_D02_C02.cost_history));
d3=datatip(h3, 'DataIndex', length(cost_D02_C03.cost_history));
d1.FontSize = 14;
d2.FontSize = 14;
d3.FontSize = 14;
hold off;
