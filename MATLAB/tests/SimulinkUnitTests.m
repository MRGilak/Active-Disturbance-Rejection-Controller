classdef SimulinkUnitTests < matlab.unittest.TestCase

    properties (TestParameter)
        models = struct(...
            'standardDiscrete1stOrder',  fullfile(SimulinkUnitTests.repoRoot(), 'Simulink', 'Standard ADRC',  'discrete_1st_order.slx'), ...
            'standardDiscrete2ndOrder',  fullfile(SimulinkUnitTests.repoRoot(), 'Simulink', 'Standard ADRC',  'discrete_2nd_order.slx'), ...
            'standardContinuous2ndOrder', fullfile(SimulinkUnitTests.repoRoot(), 'Simulink', 'Standard ADRC', 'continuous_2nd_order.slx'), ...
            'cascadedFirstOrder',        fullfile(SimulinkUnitTests.repoRoot(), 'Simulink', 'Cascaded ADRC',  'first_order.slx'), ...
            'cascadedSecondOrder',       fullfile(SimulinkUnitTests.repoRoot(), 'Simulink', 'Cascaded ADRC',  'second_order.slx'))
    end

    methods (TestClassSetup)
        function requireSimulink(tc)
            tc.assumeTrue(~isempty(ver('Simulink')), 'Simulink is not installed');
        end
    end

    methods (Test)

        function testModelLoadsAndSimulates(tc, models)
            modelFile = models;
            tc.verifyTrue(exist(modelFile, 'file') == 2, ...
                sprintf('Model file does not exist: %s', modelFile));

            [~, modelName] = fileparts(modelFile);

            load_system(modelFile);
            try
                sim(modelName, 'StopTime', '2');
            catch ME
                tc.verifyFail(sprintf('Simulation of %s failed: %s', modelName, ME.message));
            finally
                if bdIsLoaded(modelName)
                    close_system(modelName, 0);
                end
            end
        end

    end

    methods (Static)
        function root = repoRoot()
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
        end
    end

end