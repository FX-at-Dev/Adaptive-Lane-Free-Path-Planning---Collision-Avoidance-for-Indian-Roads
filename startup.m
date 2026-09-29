% STARTUP  Put the SIH 26037 stack on the path and report the capability tier.
%
%   Run this once per session (MATLAB, MATLAB Online or Octave) from the
%   repository root. It adds every source directory to the path and prints
%   which of the three access tiers described in the technical report is
%   actually available, so that a run that silently lost a toolbox is obvious
%   before it produces misleading results.

root = fileparts(mfilename('fullpath'));

addpath(fullfile(root, 'core', 'world'));
addpath(fullfile(root, 'core', 'sensors'));
addpath(fullfile(root, 'core', 'fusion'));
addpath(fullfile(root, 'core', 'predict'));
addpath(fullfile(root, 'core', 'plan'));
addpath(fullfile(root, 'core', 'decide'));
addpath(fullfile(root, 'core', 'vehicle'));
addpath(fullfile(root, 'core', 'util'));
addpath(fullfile(root, 'scenarios'));
addpath(fullfile(root, 'sim'));
addpath(fullfile(root, 'simulink'));
addpath(fullfile(root, 'matlab'));
addpath(fullfile(root, 'tests'));

fprintf('SIH 26037 -- adaptive planning stack\n');
if sih_is_octave()
    fprintf('  interpreter : GNU Octave %s (tier 0: core algorithms only)\n', OCTAVE_VERSION);
else
    fprintf('  interpreter : MATLAB %s\n', version('-release'));
    wanted = { ...
        'Simulink',                        'Simulink'; ...
        'Stateflow',                       'Stateflow'; ...
        'Automated_Driving_Toolbox',       'Automated Driving Toolbox'; ...
        'Navigation_Toolbox',              'Navigation Toolbox'; ...
        'Video_and_Image_Blockset',        'Computer Vision Toolbox'; ...
        'Neural_Network_Toolbox',          'Deep Learning Toolbox'; ...
        'Vehicle_Dynamics_Blockset',       'Vehicle Dynamics Blockset'};
    for k = 1:size(wanted, 1)
        if license('test', wanted{k,1})
            mark = 'available';
        else
            mark = 'MISSING';
        end
        fprintf('  %-28s : %s\n', wanted{k,2}, mark);
    end
end
fprintf('  run tests   : sih_test_all\n');

clear root wanted k mark
