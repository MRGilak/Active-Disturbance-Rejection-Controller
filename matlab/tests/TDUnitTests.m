classdef TDUnitTests < matlab.unittest.TestCase

    methods (Test)

        function testInvalidMethodRaises(tc)
            try
                adrc.TD('bogus_method', 0.01);
                tc.verifyFail('Expected error was not thrown');
            catch e
                tc.verifyEqual(e.identifier, '');
            end
        end

        function testAllMethodsConstructAndUpdate(tc)
            methods = {'euler', 'tod', 'ld', 'red', 'intd'};
            for i = 1:numel(methods)
                td = adrc.TD(methods{i}, 0.01);
                td.update(1);
                tc.verifyTrue(isfinite(td.y));
                tc.verifyTrue(isfinite(td.yd));
                tc.verifyTrue(isfinite(td.ydd));
            end
        end

        function testEulerTracksConstantInput(tc)
            td = adrc.TD('euler', 0.01, 0.9);
            for k = 1:500
                td.update(1);
            end
            tc.verifyEqual(td.y, 1, 'AbsTol', 1e-6);
            tc.verifyEqual(td.yd, 0, 'AbsTol', 1e-3);
        end

        function testEulerDerivativeOfRampApproximatesSlope(tc)
            td = adrc.TD('euler', 0.01, 0.9);
            for k = 1:500
                td.update(2 * (k + 1) * 0.01);
            end
            tc.verifyEqual(td.yd, 2, 'AbsTol', 0.5);
        end

        function testReset(tc)
            td = adrc.TD('euler', 0.01, 0.9);
            for k = 1:50
                td.update(1);
            end
            td.reset();
            tc.verifyEqual(td.y, 0);
            tc.verifyEqual(td.yd, 0);
            tc.verifyEqual(td.ydd, 0);
        end

        function testTODTracksConstantInput(tc)
            td = adrc.TD('tod', 0.01, 5);
            for k = 1:2000
                td.update(1);
            end
            tc.verifyEqual(td.y, 1, 'AbsTol', 0.02);
        end

        function testLDTracksConstantInput(tc)
            td = adrc.TD('ld', 0.001, 0.5);
            for k = 1:5000
                td.update(1);
            end
            tc.verifyEqual(td.y, 1, 'AbsTol', 0.02);
        end

        function testREDTracksConstantInput(tc)
            td = adrc.TD('red', 0.01, 4, 2);
            for k = 1:2000
                td.update(1);
            end
            tc.verifyEqual(td.y, 1, 'AbsTol', 0.02);
        end

        function testINTDValidParamsStayFinite(tc)
            td = adrc.TD('intd', 0.01, 1, 1, 1, 1);
            for k = 1:300
                td.update(1);
            end
            tc.verifyTrue(isfinite(td.y));
            tc.verifyTrue(isfinite(td.yd));
        end

        function testINTDInvalidParamsRaise(tc)
            try
                adrc.TD('intd', 0.01, 1, 0, 1, 1).update(1);
                tc.verifyFail('Expected error was not thrown');
            catch e
                tc.verifyEqual(e.identifier, '');
            end
            try
                adrc.TD('intd', 0.01, 1, 1, 0, 1).update(1);
                tc.verifyFail('Expected error was not thrown');
            catch e
                tc.verifyEqual(e.identifier, '');
            end
            try
                adrc.TD('intd', 0.01, 1, 1, 1, 0).update(1);
                tc.verifyFail('Expected error was not thrown');
            catch e
                tc.verifyEqual(e.identifier, '');
            end
        end

        function testMethodCaseInsensitive(tc)
            td = adrc.TD('EULER', 0.01, 0.9);
            td.update(1);
            tc.verifyEqual(td.y, 0.1, 'AbsTol', 1e-12);
        end

        function testSetParametersAndSampleTime(tc)
            td = adrc.TD('euler', 0.01, 0.9);
            td.setParameters(0.5);
            td.setSampleTime(0.001);
            td.update(1);
            tc.verifyTrue(isfinite(td.y));
        end

    end

end