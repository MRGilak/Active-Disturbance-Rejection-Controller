classdef ADRCUnitTests < matlab.unittest.TestCase

    methods (Test)

        % --- Construction / validation -----------------------------------

        function testConstructorValidOrders(tc)
            for order = 1:4
                controller = adrc.ADRC(order);
                tc.verifyEqual(controller.controllerOrder, order + 1);
            end
        end

        function testConstructorInvalidOrders(tc)
            for order = [0, -1, 1.5]
                tc.verifyError(@() adrc.ADRC(order), 'ADRC:Constructor');
            end
        end

        function testStepBeforeInitializeRaises(tc)
            controller = adrc.ADRC(2);
            tc.verifyError(@() controller.step(1, 0), 'ADRC:NotInitialized');
        end

        function testInitializeInvalidParamsRaise(tc)
            c1 = adrc.ADRC(2);
            try; c1.initialize('Tsettle', 0); tc.verifyFail('Expected error'); catch e; tc.verifyEqual(e.identifier, 'MATLAB:InputParser:ArgumentFailedValidation'); end

            c2 = adrc.ADRC(2);
            try; c2.initialize('kob', 0); tc.verifyFail('Expected error'); catch e; tc.verifyEqual(e.identifier, 'MATLAB:InputParser:ArgumentFailedValidation'); end

            c3 = adrc.ADRC(2);
            try; c3.initialize('dT', 0); tc.verifyFail('Expected error'); catch e; tc.verifyEqual(e.identifier, 'MATLAB:InputParser:ArgumentFailedValidation'); end

            c4 = adrc.ADRC(2);
            try; c4.initialize('inputDelay', -1); tc.verifyFail('Expected error'); catch e; tc.verifyEqual(e.identifier, 'MATLAB:InputParser:ArgumentFailedValidation'); end
        end

        function testInitializeInvalidStateVectorLength(tc)
            controller = adrc.ADRC(2);
            tc.verifyError(@() controller.initialize('XhatInit', [0, 0]), 'ADRC:Initialize');
        end

        % --- Closed-loop behavior -----------------------------------------

        function testFirstOrderIntegratorTracksStep(tc)
            dT = 0.01;
            controller = adrc.ADRC(1);
            controller.initialize('Tsettle', 1, 'kob', 10, 'b0', 1, 'dT', dT);

            r = 1;
            y = 0;
            for k = 1:2000
                u = controller.step(r, y);
                y = y + u * dT;
            end

            tc.verifyEqual(y, r, 'AbsTol', 0.01);
        end

        function testSecondOrderRejectsConstantDisturbance(tc)
            dT = 0.01;
            controller = adrc.ADRC(2);
            controller.initialize('Tsettle', 1, 'kob', 10, 'b0', 1, 'dT', dT);

            r = 1;
            d = 0.5;  % constant disturbance on acceleration
            p = 0;
            v = 0;
            u = 0;
            for k = 1:4000
                u = controller.step(r, p);
                v = v + (u + d) * dT;
                p = p + v * dT;
            end

            tc.verifyEqual(p, r, 'AbsTol', 0.02);
            tc.verifyTrue(isfinite(u));
        end

        function testEstimatedDisturbanceConverges(tc)
            dT = 0.01;
            controller = adrc.ADRC(1);
            controller.initialize('Tsettle', 1, 'kob', 10, 'b0', 1, 'dT', dT);

            d = 0.3;
            y = 0;
            for k = 1:2000
                u = controller.step(1, y);
                y = y + (u + d) * dT;
            end

            tc.verifyEqual(controller.getEstimatedDisturbance(), d, 'AbsTol', 0.02);
        end

        % --- Saturation ----------------------------------------------------

        function testSaturationLimitsControlInput(tc)
            controller = adrc.ADRC(1);
            controller.initialize('Tsettle', 1, 'kob', 10, 'b0', 1, ...
                                  'dT', 0.01, 'uMin', -0.5, 'uMax', 0.5);

            for k = 1:200
                u = controller.step(1, 0);
                tc.verifyLessThanOrEqual(u, 0.5 + 1e-12);
                tc.verifyGreaterThanOrEqual(u, -0.5 - 1e-12);
            end
        end

        % --- Input delay ---------------------------------------------------

        function testInputDelayRuns(tc)
            controller = adrc.ADRC(1);
            controller.initialize('Tsettle', 1, 'kob', 10, 'b0', 1, ...
                                  'dT', 0.01, 'inputDelay', 0.1);

            y = 0;
            for k = 1:500
                u = controller.step(1, y);
                y = y + u * 0.01;
            end

            tc.verifyEqual(y, 1, 'AbsTol', 0.05);
        end

        % --- Cascaded mode / misc API -------------------------------------

        function testCascadedModeReporting(tc)
            controller = adrc.ADRC(2);
            controller.initialize('omega_c', 4, 'omega_o', 40, 'b0', 1, 'dT', 0.01, ...
                'numObservers', 4, 'estimateLayers', [3; 4]);

            for k = 1:100
                controller.step(1, 0);
            end
            
            tc.verifyEqual(numel(controller.getEstimatedStates()), 3);
            selected = controller.getEstimatedStates();
            observer1 = controller.getObserverStates(1);
            observer2 = controller.getObserverStates(2);
            observer3 = controller.getObserverStates(3);
            observer4 = controller.getObserverStates(4);
            tc.verifyEqual(selected(1:2), [observer3(1); observer4(2)], 'AbsTol', 1e-12);
            tc.verifyEqual(controller.getEstimatedDisturbance(), ...
                observer1(end) + observer2(end) + observer3(end) + observer4(end), ...
                'AbsTol', 1e-12);
        end

        function testGeneralObserverAccessAndGainOverrides(tc)
            controller = adrc.ADRC(2);
            observerGains = [0.1, 0.2, 0.3; 1, 2, 3; 4, 5, 6];
            controller.initialize('omega_c', 3, 'omega_o', [20, 30, 40], ...
                'numObservers', 3, 'observerGains', observerGains, ...
                'controllerGains', [7; 8]);

            allStates = controller.getObserverStates();
            tc.verifySize(allStates, [3, 3]);
            tc.verifyEqual(controller.getObserverStates(2), allStates(:, 2));
            tc.verifyEqual(controller.getEstimatedStates(), zeros(3, 1), 'AbsTol', 1e-12);
            controller.step(1, 0);
            tc.verifyTrue(all(isfinite(controller.getEstimatedStates())));
        end

        function testResetRestoresState(tc)
            controller = adrc.ADRC(1);
            controller.initialize('Tsettle', 1, 'b0', 1, 'dT', 0.01);

            for k = 1:50
                controller.step(1, 0);
            end

            controller.reset();
            tc.verifyEqual(controller.getEstimatedStates(), zeros(2, 1), 'AbsTol', 1e-12);
        end

        function testStateVectorLengthsForAllOrders(tc)
            for order = 1:4
                controller = adrc.ADRC(order);
                controller.initialize();
                tc.verifyEqual(numel(controller.getEstimatedStates()), order + 1);
            end
        end

        function testUpdateTuningRuns(tc)
            controller = adrc.ADRC(2);
            controller.initialize('Tsettle', 1, 'kob', 10);
            controller.updateTuning(2, 5);
            u = controller.step(1, 0);
            tc.verifyTrue(isfinite(u));
        end

        function testSetTDViaInitialize(tc)
            controller = adrc.ADRC(1);
            controller.initialize('Tsettle', 1, 'b0', 1, 'dT', 0.01, ...
                                  'TD_method', 'euler', 'TD_params', {0.7});
            u = controller.step(1, 0);
            tc.verifyTrue(isfinite(u));
        end

    end

end