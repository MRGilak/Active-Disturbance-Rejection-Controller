classdef ADRC_app < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure            matlab.ui.Figure
        GridLayout          matlab.ui.container.GridLayout
        LeftPanel           matlab.ui.container.Panel
        RunButton           matlab.ui.control.Button
        UseReferenceDerivativesCheckBox  matlab.ui.control.CheckBox
        RefSignalLabel      matlab.ui.control.Label
        RefTypeDropDown     matlab.ui.control.DropDown
        RefTypeLabel        matlab.ui.control.Label
        StepTimeEditField   matlab.ui.control.NumericEditField
        StepTimeLabel       matlab.ui.control.Label
        StepAmpEditField    matlab.ui.control.NumericEditField
        StepAmpLabel        matlab.ui.control.Label
        SineFreqEditField   matlab.ui.control.NumericEditField
        SineFreqLabel       matlab.ui.control.Label
        SineAmpEditField    matlab.ui.control.NumericEditField
        SineAmpLabel        matlab.ui.control.Label
        SinePhaseEditField  matlab.ui.control.NumericEditField
        SinePhaseLabel      matlab.ui.control.Label
        SawFreqEditField    matlab.ui.control.NumericEditField
        SawFreqLabel        matlab.ui.control.Label
        SawAmpEditField     matlab.ui.control.NumericEditField
        SawAmpLabel         matlab.ui.control.Label
        uMin                matlab.ui.control.CheckBox
        uMax                matlab.ui.control.CheckBox
        uMinSlider          matlab.ui.control.Slider
        uMinSliderLabel     matlab.ui.control.Label
        uMinEditField       matlab.ui.control.NumericEditField
        uMaxSlider          matlab.ui.control.Slider
        uMaxSliderLabel     matlab.ui.control.Label
        uMaxEditField       matlab.ui.control.NumericEditField
        b0EditField         matlab.ui.control.NumericEditField
        b0EditFieldLabel    matlab.ui.control.Label
        delaySlider         matlab.ui.control.Slider
        delaySliderLabel    matlab.ui.control.Label
        delayEditField      matlab.ui.control.NumericEditField
        unmatchedDelay      matlab.ui.control.CheckBox
        ctrlDelaySlider     matlab.ui.control.Slider
        ctrlDelaySliderLabel matlab.ui.control.Label
        ctrlDelayEditField  matlab.ui.control.NumericEditField
        kobSlider           matlab.ui.control.Slider
        kobSliderLabel      matlab.ui.control.Label
        kobEditField        matlab.ui.control.NumericEditField
        TsettleSlider       matlab.ui.control.Slider
        TsettleSliderLabel  matlab.ui.control.Label
        TsettleEditField    matlab.ui.control.NumericEditField
        useDelay            matlab.ui.control.CheckBox
        CenterPanel         matlab.ui.container.Panel
        controlInput        matlab.ui.control.UIAxes
        systemOutput        matlab.ui.control.UIAxes
        RightPanel          matlab.ui.container.Panel
        TEditField          matlab.ui.control.NumericEditField
        TEditFieldLabel     matlab.ui.control.Label
        dTEditField         matlab.ui.control.NumericEditField
        dTEditFieldLabel    matlab.ui.control.Label
        dtEditField         matlab.ui.control.NumericEditField
        dtEditFieldLabel    matlab.ui.control.Label
        SystemLabel         matlab.ui.control.Label
        NumEditField        matlab.ui.control.EditField
        NumEditFieldLabel   matlab.ui.control.Label
        DenEditField        matlab.ui.control.EditField
        DenEditFieldLabel   matlab.ui.control.Label
        TDMethodDropDown    matlab.ui.control.DropDown
        TDMethodLabel       matlab.ui.control.Label
        TDParam1EditField   matlab.ui.control.NumericEditField
        TDParam1Label       matlab.ui.control.Label
        TDParam2EditField   matlab.ui.control.NumericEditField
        TDParam2Label       matlab.ui.control.Label
        TDParam3EditField   matlab.ui.control.NumericEditField
        TDParam3Label       matlab.ui.control.Label
        TDParam4EditField   matlab.ui.control.NumericEditField
        TDParam4Label       matlab.ui.control.Label
    end

    % Properties that correspond to apps with auto-reflow
    properties (Access = private)
        onePanelWidth = 576;
        twoPanelWidth = 768;
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            % Set default values
            app.dtEditField.Value = 0.001;
            app.dTEditField.Value = 0.01;
            app.TEditField.Value = 10;
            app.b0EditField.Value = 1.6;
            
            % Default system: Transfer function from demo
            % G(s) = 1.6 / (s^2 + 7.3s + 2.1)
            app.NumEditField.Value = '0 0 1.6';
            app.DenEditField.Value = '1 7.3 2.1';
            
            % TD defaults
            app.TDMethodDropDown.Value = 'tod';
            app.TDParam1EditField.Value = 1;
            app.TDParam2EditField.Value = 0;
            app.TDParam3EditField.Value = 0;
            app.TDParam4EditField.Value = 0;
            app.TsettleEditField.Value = 1;
            app.kobSlider.Value = 10;
            app.kobEditField.Value = 10;
            app.delaySlider.Value = 0;
            app.delayEditField.Value = 0;
            app.unmatchedDelay.Value = false;
            app.ctrlDelaySlider.Value = 0;
            app.ctrlDelayEditField.Value = 0;
            app.uMaxSlider.Value = 10;
            app.uMaxEditField.Value = 10;
            app.uMinSlider.Value = -10;
            app.uMinEditField.Value = -10;
            app.useDelay.Value = false;
            app.uMax.Value = false;
            app.uMin.Value = false;
            app.UseReferenceDerivativesCheckBox.Value = false;
            
            % Reference signal defaults
            app.RefTypeDropDown.Value = 'Step';
            app.StepTimeEditField.Value = 0;
            app.StepAmpEditField.Value = 1;
            app.SineFreqEditField.Value = 0.5;
            app.SineAmpEditField.Value = 1;
            app.SinePhaseEditField.Value = 0;
            app.SawFreqEditField.Value = 0.5;
            app.SawAmpEditField.Value = 1;
            
            % Show only step parameters initially
            RefTypeDropDownValueChanged(app);
            
            % Initialize TD parameter labels
            TDMethodDropDownValueChanged(app);
        end

        % Button pushed function: RunButton
        function RunButtonPushed(app, ~)
            % Parse system transfer function from UI
            try
                num = str2num(app.NumEditField.Value);
                den = str2num(app.DenEditField.Value);
                
                if isempty(num) || isempty(den)
                    uialert(app.UIFigure, 'Invalid numerator or denominator coefficients.', 'System Error');
                    return;
                end
                
                % Determine system order from denominator
                n = length(den) - 1;
                if n < 1 || n > 4
                    uialert(app.UIFigure, 'System order must be between 1 and 4.', 'System Error');
                    return;
                end
                
                % Create state-space representation
                [A, B, C, D] = tf2ss(num, den);
                
            catch ME
                uialert(app.UIFigure, ['Error parsing system: ' ME.message], 'System Error');
                return;
            end
            
            % Get simulation parameters
            dt = app.dtEditField.Value;
            T = app.TEditField.Value;
            N = round(T/dt);
            time = (dt:dt:T)';
            
            % Get controller parameters
            Tsettle = app.TsettleEditField.Value;
            kob = app.kobEditField.Value;
            bReal = app.b0EditField.Value;
            dT = app.dTEditField.Value;
            
            % Handle saturation limits
            if app.uMax.Value
                uMax_val = app.uMaxEditField.Value;
            else
                uMax_val = inf;
            end
            
            if app.uMin.Value
                uMin_val = app.uMinEditField.Value;
            else
                uMin_val = -inf;
            end
            
            % Handle input delay
            if app.useDelay.Value
                systemDelaySec = app.delayEditField.Value;
                if app.unmatchedDelay.Value
                    controllerDelaySec = app.ctrlDelayEditField.Value;
                else
                    controllerDelaySec = app.delayEditField.Value;
                end
            else
                systemDelaySec = 0;
                controllerDelaySec = 0;
            end
            
            % Get TD parameters
            useRefDerivs = app.UseReferenceDerivativesCheckBox.Value;
            TD_method = app.TDMethodDropDown.Value;
            TD_params = {};
            if app.TDParam1EditField.Value ~= 0
                TD_params{end+1} = app.TDParam1EditField.Value;
            end
            if app.TDParam2EditField.Value ~= 0
                TD_params{end+1} = app.TDParam2EditField.Value;
            end
            if app.TDParam3EditField.Value ~= 0
                TD_params{end+1} = app.TDParam3EditField.Value;
            end
            if app.TDParam4EditField.Value ~= 0
                TD_params{end+1} = app.TDParam4EditField.Value;
            end
            
            % Derive controller update rate
            samplesPerControl = max(1, round(dT/dt));
            dT_ctrl = samplesPerControl * dt;
            
            % Discretize system for simulation
            Ad = expm(A * dt);
            I = eye(size(A));
            Bd = A \ ((Ad - I) * B);
            Cd = C;
            Dd = D;
            
            % Initialize controller with ADRC class
            try
                controller = ADRC(n);
                
                % Configure TD if using reference derivatives
                if useRefDerivs
                    controller.initialize('Tsettle', Tsettle, 'kob', kob, 'b0', bReal, ...
                                        'uMin', uMin_val, 'uMax', uMax_val, ...
                                        'dT', dT_ctrl, 'inputDelay', controllerDelaySec, ...
                                        'TD_method', TD_method, 'TD_params', TD_params);
                else
                    controller.initialize('Tsettle', Tsettle, 'kob', kob, 'b0', bReal, ...
                                        'uMin', uMin_val, 'uMax', uMax_val, ...
                                        'dT', dT_ctrl, 'inputDelay', controllerDelaySec);
                end
            catch ME
                uialert(app.UIFigure, ['Error initializing controller: ' ME.message], 'Controller Error');
                return;
            end
            
            % Generate reference signal based on selection
            refType = app.RefTypeDropDown.Value;
            ref = zeros(N, 1);
            
            switch refType
                case 'Step'
                    stepTime = app.StepTimeEditField.Value;
                    stepAmp = app.StepAmpEditField.Value;
                    stepIdx = max(1, round(stepTime / dt));
                    ref(stepIdx:end) = stepAmp;
                    
                case 'Sinusoid'
                    sineFreq = app.SineFreqEditField.Value;
                    sineAmp = app.SineAmpEditField.Value;
                    sinePhase = app.SinePhaseEditField.Value;
                    w = 2*pi*sineFreq;
                    ref = sineAmp * sin(w * time + sinePhase);
                    
                case 'Sawtooth'
                    sawFreq = app.SawFreqEditField.Value;
                    sawAmp = app.SawAmpEditField.Value;
                    ref = sawAmp * sawtooth(2*pi*sawFreq * time);
            end
            
            % Initialize system arrays
            x = zeros(n, 1);
            y = zeros(N, 1);
            u = zeros(N, 1);
            nu = zeros(N, 1);
            xhat = zeros(N, n+1);
            
            % Input delay buffer for plant (if delay enabled)
            if systemDelaySec > 0
                delaySamples_sim = max(0, round(systemDelaySec / dt));
                uBuffer = repmat(0, delaySamples_sim + 1, 1);
            else
                delaySamples_sim = 0;
            end
            
            %% Simulation Loop
            for k = 1:N
                % Controller update at its own rate; hold otherwise
                if mod(k-1, samplesPerControl) == 0
                    % Use ADRC step method
                    if k == 1
                        nu(k) = controller.step(ref(k), y(k));
                    else
                        nu(k) = controller.step(ref(k), y(k-1));
                    end
                else
                    nu(k) = nu(k-1);
                end
                
                % Apply delay to plant input if enabled
                if delaySamples_sim > 0
                    u_applied = uBuffer(1);
                else
                    u_applied = nu(k);
                end
                u(k) = u_applied;
                
                % Simulate system using discrete state-space
                x = Ad * x + Bd * u_applied;
                y(k) = Cd * x + Dd * u_applied;
                
                % Update delay buffer
                if delaySamples_sim > 0
                    if numel(uBuffer) == 1
                        uBuffer(1) = nu(k);
                    else
                        uBuffer(1:end-1) = uBuffer(2:end);
                        uBuffer(end) = nu(k);
                    end
                end
                
                % Store observer states
                xhat(k, :) = controller.getEstimatedStates()';
            end
            
            % Plot results
            cla(app.systemOutput);
            hold(app.systemOutput, 'on');
            plot(app.systemOutput, time, y, 'b', 'LineWidth', 1.5);
            plot(app.systemOutput, time, ref, 'r--', 'LineWidth', 1.2);
            plot(app.systemOutput, time, xhat(:, 1), 'g', 'LineWidth', 1.2);
            grid(app.systemOutput, 'on');
            xlabel(app.systemOutput, 'time (s)');
            ylabel(app.systemOutput, '$y$', 'interpreter', 'latex');
            legend(app.systemOutput, 'Output $y$', 'Reference $r$', 'Estimate $\hat{y}$', ...
                   'interpreter', 'latex', 'Location', 'best');
            hold(app.systemOutput, 'off');
            
            cla(app.controlInput);
            hold(app.controlInput, 'on');
            if delaySamples_sim > 0
                plot(app.controlInput, time, nu, 'b', 'LineWidth', 1.2);
                plot(app.controlInput, time, u, 'r--', 'LineWidth', 1.2);
                legend(app.controlInput, 'Controller $u(t)$', 'Applied $u(t - \tau)$', ...
                       'interpreter', 'latex', 'Location', 'best');
            else
                plot(app.controlInput, time, u, 'b', 'LineWidth', 1.5);
                legend(app.controlInput, 'Control Input $u$', 'interpreter', 'latex', 'Location', 'best');
            end
            grid(app.controlInput, 'on');
            xlabel(app.controlInput, 'time (s)');
            ylabel(app.controlInput, '$u$', 'interpreter', 'latex');
            hold(app.controlInput, 'off');
        end

        % Value changed function: TsettleSlider
        function TsettleSliderValueChanged(app, ~)
            app.TsettleEditField.Value = app.TsettleSlider.Value;
        end

        % Value changed function: TsettleEditField
        function TsettleEditFieldValueChanged(app, ~)
            app.TsettleSlider.Value = app.TsettleEditField.Value;
        end

        % Value changed function: kobSlider
        function kobSliderValueChanged(app, ~)
            app.kobEditField.Value = app.kobSlider.Value;
        end

        % Value changed function: kobEditField
        function kobEditFieldValueChanged(app, ~)
            app.kobSlider.Value = app.kobEditField.Value;
        end

        % Value changed function: delaySlider
        function delaySliderValueChanged(app, ~)
            app.delayEditField.Value = app.delaySlider.Value;
        end

        % Value changed function: delayEditField
        function delayEditFieldValueChanged(app, ~)
            app.delaySlider.Value = app.delayEditField.Value;
        end

        % Value changed function: unmatchedDelay
        function unmatchedDelayValueChanged(app, ~)
            if app.unmatchedDelay.Value
                app.ctrlDelaySliderLabel.Visible = 'on';
                app.ctrlDelaySlider.Visible = 'on';
                app.ctrlDelayEditField.Visible = 'on';
            else
                app.ctrlDelaySliderLabel.Visible = 'off';
                app.ctrlDelaySlider.Visible = 'off';
                app.ctrlDelayEditField.Visible = 'off';
            end
        end

        % Value changed function: ctrlDelaySlider
        function ctrlDelaySliderValueChanged(app, ~)
            app.ctrlDelayEditField.Value = app.ctrlDelaySlider.Value;
        end

        % Value changed function: ctrlDelayEditField
        function ctrlDelayEditFieldValueChanged(app, ~)
            app.ctrlDelaySlider.Value = app.ctrlDelayEditField.Value;
        end

        % Value changed function: uMaxSlider
        function uMaxSliderValueChanged(app, ~)
            app.uMaxEditField.Value = app.uMaxSlider.Value;
        end

        % Value changed function: uMaxEditField
        function uMaxEditFieldValueChanged(app, ~)
            app.uMaxSlider.Value = app.uMaxEditField.Value;
        end

        % Value changed function: uMinSlider
        function uMinSliderValueChanged(app, ~)
            app.uMinEditField.Value = app.uMinSlider.Value;
        end

        % Value changed function: uMinEditField
        function uMinEditFieldValueChanged(app, ~)
            app.uMinSlider.Value = app.uMinEditField.Value;
        end

        % Value changed function: RefTypeDropDown
        function RefTypeDropDownValueChanged(app, ~)
            refType = app.RefTypeDropDown.Value;
            
            % Hide all reference parameter fields first
            app.StepTimeLabel.Visible = 'off';
            app.StepTimeEditField.Visible = 'off';
            app.StepAmpLabel.Visible = 'off';
            app.StepAmpEditField.Visible = 'off';
            app.SineFreqLabel.Visible = 'off';
            app.SineFreqEditField.Visible = 'off';
            app.SineAmpLabel.Visible = 'off';
            app.SineAmpEditField.Visible = 'off';
            app.SinePhaseLabel.Visible = 'off';
            app.SinePhaseEditField.Visible = 'off';
            app.SawFreqLabel.Visible = 'off';
            app.SawFreqEditField.Visible = 'off';
            app.SawAmpLabel.Visible = 'off';
            app.SawAmpEditField.Visible = 'off';
            
            % Show relevant fields based on selection
            switch refType
                case 'Step'
                    app.StepTimeLabel.Visible = 'on';
                    app.StepTimeEditField.Visible = 'on';
                    app.StepAmpLabel.Visible = 'on';
                    app.StepAmpEditField.Visible = 'on';
                case 'Sinusoid'
                    app.SineFreqLabel.Visible = 'on';
                    app.SineFreqEditField.Visible = 'on';
                    app.SineAmpLabel.Visible = 'on';
                    app.SineAmpEditField.Visible = 'on';
                    app.SinePhaseLabel.Visible = 'on';
                    app.SinePhaseEditField.Visible = 'on';
                case 'Sawtooth'
                    app.SawFreqLabel.Visible = 'on';
                    app.SawFreqEditField.Visible = 'on';
                    app.SawAmpLabel.Visible = 'on';
                    app.SawAmpEditField.Visible = 'on';
            end
        end

        % Value changed function: TDMethodDropDown
        function TDMethodDropDownValueChanged(app, ~)
            method = app.TDMethodDropDown.Value;
            
            % Hide all parameter fields and labels first
            app.TDParam1Label.Visible = 'off';
            app.TDParam1EditField.Visible = 'off';
            app.TDParam2Label.Visible = 'off';
            app.TDParam2EditField.Visible = 'off';
            app.TDParam3Label.Visible = 'off';
            app.TDParam3EditField.Visible = 'off';
            app.TDParam4Label.Visible = 'off';
            app.TDParam4EditField.Visible = 'off';
            
            switch method
                case 'euler'
                    app.TDParam1Label.Text = 'a';
                    app.TDParam1Label.Visible = 'on';
                    app.TDParam1EditField.Visible = 'on';
                    app.TDParam1EditField.Value = 0.9;
                case 'tod'
                    app.TDParam1Label.Text = 'r';
                    app.TDParam1Label.Visible = 'on';
                    app.TDParam1EditField.Visible = 'on';
                    app.TDParam1EditField.Value = 1;
                case 'ld'
                    app.TDParam1Label.Text = 'λ';
                    app.TDParam1Label.Visible = 'on';
                    app.TDParam1EditField.Visible = 'on';
                    app.TDParam1EditField.Value = 1;
                case 'red'
                    app.TDParam1Label.Text = 'λ1';
                    app.TDParam1Label.Visible = 'on';
                    app.TDParam1EditField.Visible = 'on';
                    app.TDParam1EditField.Value = 1;
                    app.TDParam2Label.Text = 'λ2';
                    app.TDParam2Label.Visible = 'on';
                    app.TDParam2EditField.Visible = 'on';
                    app.TDParam2EditField.Value = 1;
                case 'intd'
                    app.TDParam1Label.Text = 'α';
                    app.TDParam1Label.Visible = 'on';
                    app.TDParam1EditField.Visible = 'on';
                    app.TDParam1EditField.Value = 0.5;
                    app.TDParam2Label.Text = 'β';
                    app.TDParam2Label.Visible = 'on';
                    app.TDParam2EditField.Visible = 'on';
                    app.TDParam2EditField.Value = 2;
                    app.TDParam3Label.Text = 'γ';
                    app.TDParam3Label.Visible = 'on';
                    app.TDParam3EditField.Visible = 'on';
                    app.TDParam3EditField.Value = 1.5;
                    app.TDParam4Label.Text = 'r';
                    app.TDParam4Label.Visible = 'on';
                    app.TDParam4EditField.Visible = 'on';
                    app.TDParam4EditField.Value = 3;
            end
        end

        % Changes arrangement of the app based on UIFigure width
        function updateAppLayout(app, ~)
            currentFigureWidth = app.UIFigure.Position(3);
            if(currentFigureWidth <= app.onePanelWidth)
                % Change to a 3x1 grid
                app.GridLayout.RowHeight = {480, 480, 480};
                app.GridLayout.ColumnWidth = {'1x'};
                app.CenterPanel.Layout.Row = 1;
                app.CenterPanel.Layout.Column = 1;
                app.LeftPanel.Layout.Row = 2;
                app.LeftPanel.Layout.Column = 1;
                app.RightPanel.Layout.Row = 3;
                app.RightPanel.Layout.Column = 1;
            elseif (currentFigureWidth > app.onePanelWidth && currentFigureWidth <= app.twoPanelWidth)
                % Change to a 2x2 grid
                app.GridLayout.RowHeight = {480, 480};
                app.GridLayout.ColumnWidth = {'1x', '1x'};
                app.CenterPanel.Layout.Row = 1;
                app.CenterPanel.Layout.Column = [1,2];
                app.LeftPanel.Layout.Row = 2;
                app.LeftPanel.Layout.Column = 1;
                app.RightPanel.Layout.Row = 2;
                app.RightPanel.Layout.Column = 2;
            else
                % Change to a 1x3 grid
                app.GridLayout.RowHeight = {'1x'};
                app.GridLayout.ColumnWidth = {350, '1x', 200};
                app.LeftPanel.Layout.Row = 1;
                app.LeftPanel.Layout.Column = 1;
                app.CenterPanel.Layout.Row = 1;
                app.CenterPanel.Layout.Column = 2;
                app.RightPanel.Layout.Row = 1;
                app.RightPanel.Layout.Column = 3;
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.AutoResizeChildren = 'off';
            app.UIFigure.Position = [100 100 1000 700];
            app.UIFigure.Name = 'ADRC Simulation';
            app.UIFigure.SizeChangedFcn = createCallbackFcn(app, @updateAppLayout, true);

            % Create GridLayout
            app.GridLayout = uigridlayout(app.UIFigure);
            app.GridLayout.ColumnWidth = {310, '1x', 200};
            app.GridLayout.RowHeight = {'1x'};
            app.GridLayout.ColumnSpacing = 0;
            app.GridLayout.RowSpacing = 0;
            app.GridLayout.Padding = [0 0 0 0];
            app.GridLayout.Scrollable = 'on';

            % Create LeftPanel
            app.LeftPanel = uipanel(app.GridLayout);
            app.LeftPanel.Layout.Row = 1;
            app.LeftPanel.Layout.Column = 1;

            % Create RunButton
            app.RunButton = uibutton(app.LeftPanel, 'push');
            app.RunButton.ButtonPushedFcn = createCallbackFcn(app, @RunButtonPushed, true);
            app.RunButton.BackgroundColor = [0.47 0.67 0.19];
            app.RunButton.FontSize = 14;
            app.RunButton.FontWeight = 'bold';
            app.RunButton.FontColor = [1 1 1];
            app.RunButton.Position = [69 650 150 30];
            app.RunButton.Text = 'Run Simulation';

            % Create TsettleSliderLabel
            app.TsettleSliderLabel = uilabel(app.LeftPanel);
            app.TsettleSliderLabel.HorizontalAlignment = 'right';
            app.TsettleSliderLabel.Position = [7 596 40 22];
            app.TsettleSliderLabel.Text = 'Tsettle';

            % Create TsettleSlider
            app.TsettleSlider = uislider(app.LeftPanel);
            app.TsettleSlider.Limits = [0.1 10];
            app.TsettleSlider.ValueChangedFcn = createCallbackFcn(app, @TsettleSliderValueChanged, true);
            app.TsettleSlider.Position = [69 605 100 3];
            app.TsettleSlider.Value = 1;

            % Create TsettleEditField
            app.TsettleEditField = uieditfield(app.LeftPanel, 'numeric');
            app.TsettleEditField.Limits = [0.1 10];
            app.TsettleEditField.ValueChangedFcn = createCallbackFcn(app, @TsettleEditFieldValueChanged, true);
            app.TsettleEditField.Position = [180 596 50 22];
            app.TsettleEditField.Value = 1;

            % Create kobSliderLabel
            app.kobSliderLabel = uilabel(app.LeftPanel);
            app.kobSliderLabel.HorizontalAlignment = 'right';
            app.kobSliderLabel.Position = [22 552 25 22];
            app.kobSliderLabel.Text = 'kob';

            % Create kobSlider
            app.kobSlider = uislider(app.LeftPanel);
            app.kobSlider.Limits = [1 50];
            app.kobSlider.ValueChangedFcn = createCallbackFcn(app, @kobSliderValueChanged, true);
            app.kobSlider.Position = [69 561 100 3];
            app.kobSlider.Value = 10;

            % Create kobEditField
            app.kobEditField = uieditfield(app.LeftPanel, 'numeric');
            app.kobEditField.Limits = [1 50];
            app.kobEditField.ValueChangedFcn = createCallbackFcn(app, @kobEditFieldValueChanged, true);
            app.kobEditField.Position = [180 552 50 22];
            app.kobEditField.Value = 10;

            % Create b0EditFieldLabel
            app.b0EditFieldLabel = uilabel(app.LeftPanel);
            app.b0EditFieldLabel.HorizontalAlignment = 'right';
            app.b0EditFieldLabel.Position = [22 495 25 22];
            app.b0EditFieldLabel.Text = 'b0';

            % Create b0EditField
            app.b0EditField = uieditfield(app.LeftPanel, 'numeric');
            app.b0EditField.Position = [62 495 100 22];
            app.b0EditField.Value = 1.6;

            % Create delaySliderLabel
            app.delaySliderLabel = uilabel(app.LeftPanel);
            app.delaySliderLabel.HorizontalAlignment = 'right';
            app.delaySliderLabel.Position = [13 458 34 22];
            app.delaySliderLabel.Text = 'delay';

            % Create delaySlider
            app.delaySlider = uislider(app.LeftPanel);
            app.delaySlider.Limits = [0 2];
            app.delaySlider.ValueChangedFcn = createCallbackFcn(app, @delaySliderValueChanged, true);
            app.delaySlider.Position = [69 467 100 3];
            app.delaySlider.Value = 0;

            % Create delayEditField
            app.delayEditField = uieditfield(app.LeftPanel, 'numeric');
            app.delayEditField.Limits = [0 2];
            app.delayEditField.ValueChangedFcn = createCallbackFcn(app, @delayEditFieldValueChanged, true);
            app.delayEditField.Position = [180 458 50 22];
            app.delayEditField.Value = 0;

            % Create useDelay
            app.useDelay = uicheckbox(app.LeftPanel);
            app.useDelay.Text = 'Enable';
            app.useDelay.Position = [240 458 80 22];
            app.useDelay.Value = false;

            % Create unmatchedDelay
            app.unmatchedDelay = uicheckbox(app.LeftPanel);
            app.unmatchedDelay.Text = 'Unmatched delay';
            app.unmatchedDelay.ValueChangedFcn = createCallbackFcn(app, @unmatchedDelayValueChanged, true);
            app.unmatchedDelay.Position = [13 415 140 22];
            app.unmatchedDelay.Value = false;

            % Create ctrlDelaySliderLabel
            app.ctrlDelaySliderLabel = uilabel(app.LeftPanel);
            app.ctrlDelaySliderLabel.HorizontalAlignment = 'right';
            app.ctrlDelaySliderLabel.Position = [3 390 110 22];
            app.ctrlDelaySliderLabel.Text = 'Controller Delay';
            app.ctrlDelaySliderLabel.Visible = 'off';

            % Create ctrlDelaySlider
            app.ctrlDelaySlider = uislider(app.LeftPanel);
            app.ctrlDelaySlider.Limits = [0 2];
            app.ctrlDelaySlider.ValueChangedFcn = createCallbackFcn(app, @ctrlDelaySliderValueChanged, true);
            app.ctrlDelaySlider.Position = [125 399 140 3];
            app.ctrlDelaySlider.Value = 0;
            app.ctrlDelaySlider.Visible = 'off';

            % Create ctrlDelayEditField
            app.ctrlDelayEditField = uieditfield(app.LeftPanel, 'numeric');
            app.ctrlDelayEditField.Limits = [0 2];
            app.ctrlDelayEditField.ValueChangedFcn = createCallbackFcn(app, @ctrlDelayEditFieldValueChanged, true);
            app.ctrlDelayEditField.Position = [275 390 40 22];
            app.ctrlDelayEditField.Value = 0;
            app.ctrlDelayEditField.Visible = 'off';

            % Create uMaxSliderLabel
            app.uMaxSliderLabel = uilabel(app.LeftPanel);
            app.uMaxSliderLabel.HorizontalAlignment = 'right';
            app.uMaxSliderLabel.Position = [13 340 44 22];
            app.uMaxSliderLabel.Text = 'uMax';

            % Create uMaxSlider
            app.uMaxSlider = uislider(app.LeftPanel);
            app.uMaxSlider.Limits = [-50 50];
            app.uMaxSlider.ValueChangedFcn = createCallbackFcn(app, @uMaxSliderValueChanged, true);
            app.uMaxSlider.Position = [69 349 120 3];
            app.uMaxSlider.Value = 10;

            % Create uMaxEditField
            app.uMaxEditField = uieditfield(app.LeftPanel, 'numeric');
            app.uMaxEditField.Limits = [-50 50];
            app.uMaxEditField.ValueChangedFcn = createCallbackFcn(app, @uMaxEditFieldValueChanged, true);
            app.uMaxEditField.Position = [200 340 40 22];
            app.uMaxEditField.Value = 10;

            % Create uMax
            app.uMax = uicheckbox(app.LeftPanel);
            app.uMax.Text = 'Enable';
            app.uMax.Position = [245 340 90 22];
            app.uMax.Value = false;

            % Create uMinSliderLabel
            app.uMinSliderLabel = uilabel(app.LeftPanel);
            app.uMinSliderLabel.HorizontalAlignment = 'right';
            app.uMinSliderLabel.Position = [15 298 31 22];
            app.uMinSliderLabel.Text = 'uMin';

            % Create uMinSlider
            app.uMinSlider = uislider(app.LeftPanel);
            app.uMinSlider.Limits = [-50 50];
            app.uMinSlider.ValueChangedFcn = createCallbackFcn(app, @uMinSliderValueChanged, true);
            app.uMinSlider.Position = [68 307 120 3];
            app.uMinSlider.Value = -10;

            % Create uMinEditField
            app.uMinEditField = uieditfield(app.LeftPanel, 'numeric');
            app.uMinEditField.Limits = [-50 50];
            app.uMinEditField.ValueChangedFcn = createCallbackFcn(app, @uMinEditFieldValueChanged, true);
            app.uMinEditField.Position = [200 298 40 22];
            app.uMinEditField.Value = -10;

            % Create uMin
            app.uMin = uicheckbox(app.LeftPanel);
            app.uMin.Text = 'Enable';
            app.uMin.Position = [245 298 90 22];
            app.uMin.Value = false;

            % Create RefSignalLabel
            app.RefSignalLabel = uilabel(app.LeftPanel);
            app.RefSignalLabel.FontWeight = 'bold';
            app.RefSignalLabel.Position = [13 241 150 22];
            app.RefSignalLabel.Text = 'Reference Signal';

            % Create RefTypeLabel
            app.RefTypeLabel = uilabel(app.LeftPanel);
            app.RefTypeLabel.HorizontalAlignment = 'right';
            app.RefTypeLabel.Position = [13 217 30 22];
            app.RefTypeLabel.Text = 'Type';

            % Create RefTypeDropDown
            app.RefTypeDropDown = uidropdown(app.LeftPanel);
            app.RefTypeDropDown.Items = {'Step', 'Sinusoid', 'Sawtooth'};
            app.RefTypeDropDown.ValueChangedFcn = createCallbackFcn(app, @RefTypeDropDownValueChanged, true);
            app.RefTypeDropDown.Position = [58 217 100 22];
            app.RefTypeDropDown.Value = 'Step';

            % Create Step parameters
            app.StepTimeLabel = uilabel(app.LeftPanel);
            app.StepTimeLabel.HorizontalAlignment = 'right';
            app.StepTimeLabel.Position = [170 217 30 22];
            app.StepTimeLabel.Text = 'Time';

            app.StepTimeEditField = uieditfield(app.LeftPanel, 'numeric');
            app.StepTimeEditField.Position = [210 217 50 22];
            app.StepTimeEditField.Value = 0;

            app.StepAmpLabel = uilabel(app.LeftPanel);
            app.StepAmpLabel.HorizontalAlignment = 'right';
            app.StepAmpLabel.Position = [170 197 30 22];
            app.StepAmpLabel.Text = 'Amp';

            app.StepAmpEditField = uieditfield(app.LeftPanel, 'numeric');
            app.StepAmpEditField.Position = [210 197 50 22];
            app.StepAmpEditField.Value = 1;

            % Create Sinusoid parameters
            app.SineFreqLabel = uilabel(app.LeftPanel);
            app.SineFreqLabel.HorizontalAlignment = 'right';
            app.SineFreqLabel.Position = [170 217 30 22];
            app.SineFreqLabel.Text = 'Freq';
            app.SineFreqLabel.Visible = 'off';

            app.SineFreqEditField = uieditfield(app.LeftPanel, 'numeric');
            app.SineFreqEditField.Position = [210 217 50 22];
            app.SineFreqEditField.Value = 0.5;
            app.SineFreqEditField.Visible = 'off';

            app.SineAmpLabel = uilabel(app.LeftPanel);
            app.SineAmpLabel.HorizontalAlignment = 'right';
            app.SineAmpLabel.Position = [170 197 30 22];
            app.SineAmpLabel.Text = 'Amp';
            app.SineAmpLabel.Visible = 'off';

            app.SineAmpEditField = uieditfield(app.LeftPanel, 'numeric');
            app.SineAmpEditField.Position = [210 197 50 22];
            app.SineAmpEditField.Value = 1;
            app.SineAmpEditField.Visible = 'off';

            app.SinePhaseLabel = uilabel(app.LeftPanel);
            app.SinePhaseLabel.HorizontalAlignment = 'right';
            app.SinePhaseLabel.Position = [170 177 35 22];
            app.SinePhaseLabel.Text = 'Phase';
            app.SinePhaseLabel.Visible = 'off';

            app.SinePhaseEditField = uieditfield(app.LeftPanel, 'numeric');
            app.SinePhaseEditField.Position = [210 177 50 22];
            app.SinePhaseEditField.Value = 0;
            app.SinePhaseEditField.Visible = 'off';

            % Create Sawtooth parameters
            app.SawFreqLabel = uilabel(app.LeftPanel);
            app.SawFreqLabel.HorizontalAlignment = 'right';
            app.SawFreqLabel.Position = [170 217 30 22];
            app.SawFreqLabel.Text = 'Freq';
            app.SawFreqLabel.Visible = 'off';

            app.SawFreqEditField = uieditfield(app.LeftPanel, 'numeric');
            app.SawFreqEditField.Position = [210 217 50 22];
            app.SawFreqEditField.Value = 0.5;
            app.SawFreqEditField.Visible = 'off';

            app.SawAmpLabel = uilabel(app.LeftPanel);
            app.SawAmpLabel.HorizontalAlignment = 'right';
            app.SawAmpLabel.Position = [170 197 30 22];
            app.SawAmpLabel.Text = 'Amp';
            app.SawAmpLabel.Visible = 'off';

            app.SawAmpEditField = uieditfield(app.LeftPanel, 'numeric');
            app.SawAmpEditField.Position = [210 197 50 22];
            app.SawAmpEditField.Value = 1;
            app.SawAmpEditField.Visible = 'off';

            % Create UseReferenceDerivativesCheckBox
            app.UseReferenceDerivativesCheckBox = uicheckbox(app.LeftPanel);
            app.UseReferenceDerivativesCheckBox.Text = 'Use Reference Derivatives';
            app.UseReferenceDerivativesCheckBox.Position = [13 167 200 22];
            app.UseReferenceDerivativesCheckBox.Value = false;

            % Create CenterPanel
            app.CenterPanel = uipanel(app.GridLayout);
            app.CenterPanel.Layout.Row = 1;
            app.CenterPanel.Layout.Column = 2;

            % Create systemOutput
            app.systemOutput = uiaxes(app.CenterPanel);
            title(app.systemOutput, 'System Output')
            xlabel(app.systemOutput, 'time (s)')
            ylabel(app.systemOutput, 'y')
            app.systemOutput.Position = [7 451 413 238];

            % Create controlInput
            app.controlInput = uiaxes(app.CenterPanel);
            title(app.controlInput, 'Control Input')
            xlabel(app.controlInput, 'time (s)')
            ylabel(app.controlInput, 'u')
            app.controlInput.Position = [7 229 413 214];

            % Create RightPanel
            app.RightPanel = uipanel(app.GridLayout);
            app.RightPanel.Layout.Row = 1;
            app.RightPanel.Layout.Column = 3;

            % Create dtEditFieldLabel
            app.dtEditFieldLabel = uilabel(app.RightPanel);
            app.dtEditFieldLabel.HorizontalAlignment = 'right';
            app.dtEditFieldLabel.Position = [28 586 25 22];
            app.dtEditFieldLabel.Text = 'dt';

            % Create dtEditField
            app.dtEditField = uieditfield(app.RightPanel, 'numeric');
            app.dtEditField.Position = [67 586 63 22];
            app.dtEditField.Value = 0.001;

            % Create dTEditFieldLabel
            app.dTEditFieldLabel = uilabel(app.RightPanel);
            app.dTEditFieldLabel.HorizontalAlignment = 'right';
            app.dTEditFieldLabel.Position = [28 541 25 22];
            app.dTEditFieldLabel.Text = 'dT';

            % Create dTEditField
            app.dTEditField = uieditfield(app.RightPanel, 'numeric');
            app.dTEditField.Position = [67 541 63 22];
            app.dTEditField.Value = 0.01;

            % Create TEditFieldLabel
            app.TEditFieldLabel = uilabel(app.RightPanel);
            app.TEditFieldLabel.HorizontalAlignment = 'right';
            app.TEditFieldLabel.Position = [28 496 25 22];
            app.TEditFieldLabel.Text = 'T';

            % Create TEditField
            app.TEditField = uieditfield(app.RightPanel, 'numeric');
            app.TEditField.Position = [67 496 63 22];
            app.TEditField.Value = 10;

            % Create SystemLabel
            app.SystemLabel = uilabel(app.RightPanel);
            app.SystemLabel.FontWeight = 'bold';
            app.SystemLabel.Position = [11 450 180 22];
            app.SystemLabel.Text = 'System Transfer Function';

            % Create NumEditFieldLabel
            app.NumEditFieldLabel = uilabel(app.RightPanel);
            app.NumEditFieldLabel.HorizontalAlignment = 'right';
            app.NumEditFieldLabel.Position = [11 420 30 22];
            app.NumEditFieldLabel.Text = 'Num';

            % Create NumEditField
            app.NumEditField = uieditfield(app.RightPanel, 'text');
            app.NumEditField.Position = [50 420 140 22];
            app.NumEditField.Value = '0 0 1.6';

            % Create DenEditFieldLabel
            app.DenEditFieldLabel = uilabel(app.RightPanel);
            app.DenEditFieldLabel.HorizontalAlignment = 'right';
            app.DenEditFieldLabel.Position = [11 385 30 22];
            app.DenEditFieldLabel.Text = 'Den';

            % Create DenEditField
            app.DenEditField = uieditfield(app.RightPanel, 'text');
            app.DenEditField.Position = [50 385 140 22];
            app.DenEditField.Value = '1 7.3 2.1';

            % Create TDMethodLabel
            app.TDMethodLabel = uilabel(app.RightPanel);
            app.TDMethodLabel.HorizontalAlignment = 'right';
            app.TDMethodLabel.Position = [11 350 60 22];
            app.TDMethodLabel.Text = 'TD Method';

            % Create TDMethodDropDown
            app.TDMethodDropDown = uidropdown(app.RightPanel);
            app.TDMethodDropDown.Items = {'euler', 'tod', 'ld', 'red', 'intd'};
            app.TDMethodDropDown.ValueChangedFcn = createCallbackFcn(app, @TDMethodDropDownValueChanged, true);
            app.TDMethodDropDown.Position = [80 350 80 22];
            app.TDMethodDropDown.Value = 'tod';

            % Create TDParam1Label
            app.TDParam1Label = uilabel(app.RightPanel);
            app.TDParam1Label.HorizontalAlignment = 'right';
            app.TDParam1Label.Position = [11 315 35 22];
            app.TDParam1Label.Text = 'P1';

            % Create TDParam1EditField
            app.TDParam1EditField = uieditfield(app.RightPanel, 'numeric');
            app.TDParam1EditField.Position = [50 315 40 22];
            app.TDParam1EditField.Value = 1;

            % Create TDParam2Label
            app.TDParam2Label = uilabel(app.RightPanel);
            app.TDParam2Label.HorizontalAlignment = 'right';
            app.TDParam2Label.Position = [95 315 15 22];
            app.TDParam2Label.Text = 'P2';

            % Create TDParam2EditField
            app.TDParam2EditField = uieditfield(app.RightPanel, 'numeric');
            app.TDParam2EditField.Position = [115 315 40 22];
            app.TDParam2EditField.Value = 0;

            % Create TDParam3Label
            app.TDParam3Label = uilabel(app.RightPanel);
            app.TDParam3Label.HorizontalAlignment = 'right';
            app.TDParam3Label.Position = [11 280 15 22];
            app.TDParam3Label.Text = 'P3';

            % Create TDParam3EditField
            app.TDParam3EditField = uieditfield(app.RightPanel, 'numeric');
            app.TDParam3EditField.Position = [30 280 40 22];
            app.TDParam3EditField.Value = 0;

            % Create TDParam4Label
            app.TDParam4Label = uilabel(app.RightPanel);
            app.TDParam4Label.HorizontalAlignment = 'right';
            app.TDParam4Label.Position = [75 280 15 22];
            app.TDParam4Label.Text = 'P4';

            % Create TDParam4EditField
            app.TDParam4EditField = uieditfield(app.RightPanel, 'numeric');
            app.TDParam4EditField.Position = [95 280 40 22];
            app.TDParam4EditField.Value = 0;

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = ADRC_app

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end
