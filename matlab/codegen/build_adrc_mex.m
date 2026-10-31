function build_adrc_mex()
%BUILD_ADRC_MEX Generate MEX and C library artifacts for ADRCMex.

thisDir = fileparts(mfilename('fullpath'));
outputDir = fullfile(thisDir, 'generated');
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

config = adrc_codegen_config();
args = {uint8(0), 0.0, 0.0, zeros(4, 1), 0, config};

mexCfg = coder.config('mex');
mexCfg.GenerateReport = true;
codegen('-config', mexCfg, '-d', outputDir, ...
    '-args', args, fullfile(thisDir, 'adrc_codegen.m'));

libCfg = coder.config('lib');
libCfg.GenerateReport = true;
libCfg.GenCodeOnly = true;
libDir = fullfile(outputDir, 'c');
codegen('-config', libCfg, '-d', libDir, ...
    '-args', args, fullfile(thisDir, 'adrc_codegen.m'));

fprintf('Generated MEX and C library artifacts in:\n%s\n', outputDir);
end
