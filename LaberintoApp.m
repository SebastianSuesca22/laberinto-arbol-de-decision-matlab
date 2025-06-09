classdef LaberintoApp < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure        matlab.ui.Figure
        UIAxes         matlab.ui.control.UIAxes
        UIAxesPath     matlab.ui.control.UIAxes
        SpinnerStartX  matlab.ui.control.Spinner
        SpinnerStartY  matlab.ui.control.Spinner
        SpinnerEndX    matlab.ui.control.Spinner
        SpinnerEndY    matlab.ui.control.Spinner
        StartButton    matlab.ui.control.Button
        RestartButton  matlab.ui.control.Button
        StartLabel     matlab.ui.control.Label
        EndLabel       matlab.ui.control.Label
        PathLabel      matlab.ui.control.Label
    end

    % App initialization and construction
    methods (Access = public)

        % Construct app
        function app = LaberintoApp()

            % Create and configure components
            createComponents(app)
        end

        % A* Algorithm for pathfinding
        function path = astar(app, binaryMatrix, startNode, endNode)
            % Initialize parameters
            [rows, cols] = size(binaryMatrix);
            openSet = startNode;
            cameFrom = containers.Map('KeyType', 'char', 'ValueType', 'any');
            gScore = inf(rows, cols);
            gScore(startNode(1), startNode(2)) = 0;
            fScore = inf(rows, cols);
            fScore(startNode(1), startNode(2)) = heuristic(startNode, endNode);

            % Helper function to calculate heuristic (Manhattan distance)
            function h = heuristic(node, goal)
                h = abs(node(1) - goal(1)) + abs(node(2) - goal(2));
            end

            % Reconstruct path
            function p = reconstructPath(cameFrom, current)
                p = current;
                while cameFrom.isKey(mat2str(current))
                    current = cameFrom(mat2str(current));
                    p = [current; p];
                end
            end

            % A* loop
            while ~isempty(openSet)
                % Get node in openSet with lowest fScore
                [~, idx] = min(arrayfun(@(n) fScore(openSet(n, 1), openSet(n, 2)), 1:size(openSet, 1)));
                current = openSet(idx, :);

                % If reached the goal
                if isequal(current, endNode)
                    path = reconstructPath(cameFrom, current);
                    return;
                end

                % Remove current from openSet
                openSet(idx, :) = [];

                % Evaluate neighbors
                neighbors = [0 1; 1 0; 0 -1; -1 0; 1 1; -1 -1; 1 -1; -1 1] + current;
                for i = 1:size(neighbors, 1)
                    neighbor = neighbors(i, :);

                    % Validate neighbor is within bounds and not an obstacle
                    if neighbor(1) > 0 && neighbor(1) <= rows && neighbor(2) > 0 && neighbor(2) <= cols && binaryMatrix(neighbor(1), neighbor(2)) == 0
                        % Check diagonal movements to avoid cutting corners
                        if abs(neighbor(1) - current(1)) == 1 && abs(neighbor(2) - current(2)) == 1
                            if binaryMatrix(current(1), neighbor(2)) == 1 || binaryMatrix(neighbor(1), current(2)) == 1
                                continue;
                            end
                        end

                        tentativeGScore = gScore(current(1), current(2)) + sqrt((neighbor(1) - current(1))^2 + (neighbor(2) - current(2))^2);
                        if tentativeGScore < gScore(neighbor(1), neighbor(2))
                            cameFrom(mat2str(neighbor)) = current;
                            gScore(neighbor(1), neighbor(2)) = tentativeGScore;
                            fScore(neighbor(1), neighbor(2)) = gScore(neighbor(1), neighbor(2)) + heuristic(neighbor, endNode);
                            if ~ismember(neighbor, openSet, 'rows')
                                openSet = [openSet; neighbor];
                            end
                        end
                    end
                end
            end

            % No path found
            path = [];
        end

        % Callback for start button
        function startCallback(app, ~)
            % Datos del laberinto
            Data = [
                1  1 1 0   0 0 1    1  1    1    1   1  1;  
                1    0.7 0.4 0   0 0 1    1  0    0.5  1   0.6  0.6; 
                1    0.5 0.3 0   0 0 0.5  0  0.4  0    0   0.3  0.6; 
                0.5  0.3 0.2 0   1 0 0    0  0    0.5  1   0.3  0.6;
                0.3  0.1 0.1 0   1 0 0    1  0    0.5  1   0.2  0.6;
                0.2  0.5 0.2 0   1 0 1    1  0    0.5  1   0.2  0.6;
                0.1  0.5 0.3 0   0 0 1    1  0    0.5  1   0.1  0.6;
                0.1  0.3 0.4 0   0 0 1    1  0    0.5  1   0.1  0.6;
                0.1  0.5 0.5 1   1 0 1    1  0.3  0.5  1   0.1  0.6;
                1    0.7 0.4 0   1 0 1    1  0    0.5  1   0.1  0.6; 
                1    0.5 0.3 0   1 0 1    1  0.4  0    1   0.2  0.6; 
                0.5  0.3 0.2 0   1 0 1    1  0    0.5  1   0.2  0.6;
                0.3  0.1 0.1 0   1 0 1    1  0    0.5  1   0.2  0.6;
                0.2  0.5 0.2 0   1 0 1    1  0    0.5  1   0.4  0.6;
                0.1  0.5 0.3 0   0 0 1    1  0    0.5  0.3 0.4  0.6;
                0.1  0.5 0.4 0.6 0 0 1    1  0    0.5  0.3 0.6  0.6;
                0.1  0.5 0.4 0.4 0 0 0    1  0.3  0.5  0.3 0.3  0.3;
                0.2  0.5 0.5 0   0 0 0.6  1  0    0.5  1   0.6  0.6
            ];

            % Crear un mapa binario
            binaryMatrix = Data > 0.4;
            [fD, cD] = size(Data);
            worldMap = binaryOccupancyMap(binaryMatrix, 1);
            worldMap.LocalOriginInWorld = [1 1];

            % Mostrar el mapa binario en el primer eje
            show(worldMap, 'Parent', app.UIAxes);
            hold(app.UIAxes, 'on');
            app.UIAxes.Title.String = 'Laberinto';

            % Leer coordenadas de inicio y fin
            startX = round(app.SpinnerStartX.Value);
            startY = round(app.SpinnerStartY.Value);
            endX = round(app.SpinnerEndX.Value);
            endY = round(app.SpinnerEndY.Value);

            % Validar coordenadas dentro de los límites de la matriz
            if startX < 1 || startX > cD || startY < 1 || startY > fD || ...
               endX < 1 || endX > cD || endY < 1 || endY > fD
                uialert(app.UIFigure, 'Coordenadas fuera de rango. Elija valores entre 1 y el tamaño del mapa.', 'Error');
                return;
            end

            % Validar celdas transitables
            if binaryMatrix(startY, startX) == 1 || binaryMatrix(endY, endX) == 1
                uialert(app.UIFigure, 'Coordenadas inválidas. Elija celdas transitables.', 'Error');
                return;
            end

            % Depuración: Mostrar binaryMatrix en consola
            disp('Mapa binario:');
            disp(binaryMatrix);

            % Calcular el camino usando A*
            startNode = [startY, startX];
            endNode = [endY, endX];
            path = astar(app, binaryMatrix, startNode, endNode);

            % Si no se encontró un camino
            if isempty(path)
                uialert(app.UIFigure, 'No se encontró un camino válido.', 'Error');
                return;
            end

            % Mostrar el recorrido en ambos ejes
            % Limpiar el segundo eje y preparar para graficar
            cla(app.UIAxesPath);
            hold(app.UIAxesPath, 'on');
            app.UIAxesPath.Title.String = 'Recorrido';
            app.UIAxesPath.XLabel.String = 'X [meters]';
            app.UIAxesPath.YLabel.String = 'Y [meters]';

            % Trazar el recorrido completo en ambos ejes
            for i = 2:size(path, 1)
                % Graficar en el primer eje (Laberinto)
                plot(app.UIAxes, [path(i-1, 2), path(i, 2)], [path(i-1, 1), path(i, 1)], 'g-', 'LineWidth', 1.5);
                plot(app.UIAxes, path(i, 2), path(i, 1), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
                % Graficar en el segundo eje (Recorrido)
                plot(app.UIAxesPath, [path(i-1, 2), path(i, 2)], [path(i-1, 1), path(i, 1)], 'r-', 'LineWidth', 1.5);
                plot(app.UIAxesPath, path(i, 2), path(i, 1), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
                drawnow;
                pause(0.1);
            end

            % Finalizar mostrando el punto de inicio y destino en ambos ejes
            % En el primer eje (Laberinto)
            plot(app.UIAxes, startX, startY, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
            plot(app.UIAxes, endX, endY, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
            % En el segundo eje (Recorrido)
            plot(app.UIAxesPath, startX, startY, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
            plot(app.UIAxesPath, endX, endY, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

            hold(app.UIAxesPath, 'off');
        end

        % Callback for restart button
        function restartCallback(app, ~)
            % Limpiar ambos ejes
            cla(app.UIAxes);
            cla(app.UIAxesPath);
            app.UIAxes.Title.String = '';
            app.UIAxesPath.Title.String = '';

            % Reiniciar valores de los spinners
            app.SpinnerStartX.Value = 1;
            app.SpinnerStartY.Value = 1;
            app.SpinnerEndX.Value = 1;
            app.SpinnerEndY.Value = 1;
        end

        % Create and configure components
        function createComponents(app)

            % Create UIFigure
            app.UIFigure = uifigure('Visible', 'on');
            app.UIFigure.Name = 'Laberinto App';
            app.UIFigure.Position = [100 100 900 600];
            app.UIFigure.Color = [0.9, 0.9, 0.98]; % Fondo pastel

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            app.UIAxes.Position = [50 300 350 250];

            % Create UIAxesPath
            app.UIAxesPath = uiaxes(app.UIFigure);
            app.UIAxesPath.Position = [450 300 350 250];

            % Create Spinners for start coordinates
            app.SpinnerStartX = uispinner(app.UIFigure, 'Position', [150, 200, 120, 22], 'Limits', [1, 18], 'ValueDisplayFormat', 'Inicio X: %.0f');
            app.SpinnerStartY = uispinner(app.UIFigure, 'Position', [150, 170, 120, 22], 'Limits', [1, 18], 'ValueDisplayFormat', 'Inicio Y: %.0f');

            % Create Spinners for end coordinates
            app.SpinnerEndX = uispinner(app.UIFigure, 'Position', [550, 200, 120, 22], 'Limits', [1, 18], 'ValueDisplayFormat', 'Fin X: %.0f');
            app.SpinnerEndY = uispinner(app.UIFigure, 'Position', [550, 170, 120, 22], 'Limits', [1, 18], 'ValueDisplayFormat', 'Fin Y: %.0f');

            % Create Start Button
            app.StartButton = uibutton(app.UIFigure, 'push', 'Position', [250, 120, 150, 50], 'Text', 'Iniciar', 'FontSize', 16, 'FontWeight', 'bold', 'ButtonPushedFcn', @(btn, event) startCallback(app));

            % Create Restart Button
            app.RestartButton = uibutton(app.UIFigure, 'push', 'Position', [500, 120, 150, 50], 'Text', 'Reiniciar', 'FontSize', 16, 'FontWeight', 'bold', 'ButtonPushedFcn', @(btn, event) restartCallback(app));

            % Add labels
            app.StartLabel = uilabel(app.UIFigure, 'Position', [50, 230, 200, 22], 'Text', 'Coordenadas de Inicio:', 'FontWeight', 'bold');
            app.EndLabel = uilabel(app.UIFigure, 'Position', [450, 230, 200, 22], 'Text', 'Coordenadas de Destino:', 'FontWeight', 'bold');
        end
    end
end
