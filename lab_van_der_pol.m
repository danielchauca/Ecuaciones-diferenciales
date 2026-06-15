function lab_van_der_pol()
    % --- Parámetros del problema ---
    mu = 1.5;           % Coeficiente de amortiguamiento no lineal
    t0 = 0;             % Tiempo inicial
    tf = 20;            % Tiempo final
    h = 0.05;           % Tamaño de paso original (puedes cambiarlo para Q3)
    
    % --- Condiciones Iniciales ---
    % x(0) = 2.0 (desplazamiento), v(0) = 0 (velocidad)
    Y = [2.0; 0.0]; 
    
    % --- Vector de tiempo ---
    t = t0:h:tf;
    N = length(t);
    
    % --- Matrices para almacenar los resultados ---
    % Cada columna guardará [x; v] para cada instante de tiempo
    resultados = zeros(2, N);
    resultados(:, 1) = Y;
    
    % --- Función del sistema de EDOs ---
    % y(1) es x, y(2) es v
    f = @(t, y) [y(2); mu * (1 - y(1)^2) * y(2) - y(1)];
    
    % --- Bucle del Método RK4 ---
    for i = 1:(N-1)
        ti = t(i);
        yi = resultados(:, i);
        
        k1 = f(ti, yi);
        k2 = f(ti + h/2, yi + (h/2)*k1);
        k3 = f(ti + h/2, yi + (h/2)*k2);
        k4 = f(ti + h, yi + h*k3);
        
        % Actualización de los estados
        resultados(:, i+1) = yi + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
    end
    
    x = resultados(1, :);
    v = resultados(2, :);
    
    fprintf('=== PRIMERAS 10 ITERACIONES ===\n');
    fprintf('%-10s %-15s %-15s\n', 'Iter (t)', 'Desplazamiento(x)', 'Velocidad(v)');
    for i = 1:11 
        fprintf('%-10.2f %-15.6f %-15.6f\n', t(i), x(i), v(i));
    end
    
    fprintf('\n=== ESTADOS FINALES ===\n');
    fprintf('%-10s %-15s %-15s\n', 'Iter (t)', 'Desplazamiento(x)', 'Velocidad(v)');
    for i = (N-2):N
        fprintf('%-10.2f %-15.6f %-15.6f\n', t(i), x(i), v(i));
    end
    
    figure(1);
    plot(t, x, '-b', 'LineWidth', 2); hold on;
    plot(t, v, '--r', 'LineWidth', 2);
    grid on;
    xlabel('Tiempo t (s)');
    ylabel('Amplitud');
    title('Gráfica 1: Evolución Temporal de x(t) y v(t)');
    legend('Desplazamiento x(t)', 'Velocidad v(t)');
    hold off;
   
    figure(2);
    plot(x, v, '-g', 'LineWidth', 2); hold on;
    plot(x(1), v(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8); % Punto inicial
    grid on;
    xlabel('Desplazamiento x');
    ylabel('Velocidad v');
    title('Gráfica 2: Diagrama de Espacio de Fases (Ciclo Límite)');
    legend('Trayectoria', 'Inicio (2,0)');
    hold off;
end