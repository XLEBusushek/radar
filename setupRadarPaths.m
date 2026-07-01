function setupRadarPaths()
    % setupRadarPaths  Добавляет каталоги проекта в MATLAB path.

    projectRoot = fileparts(mfilename('fullpath'));

    addpath(fullfile(projectRoot, 'enums'));
    addpath(fullfile(projectRoot, 'models'));
    addpath(fullfile(projectRoot, 'profiles'));
    addpath(fullfile(projectRoot, 'factory'));
    addpath(fullfile(projectRoot, 'trajectory'));
    addpath(fullfile(projectRoot, 'simulation'));
    addpath(fullfile(projectRoot, 'integration'));
    addpath(fullfile(projectRoot, 'export'));
    addpath(fullfile(projectRoot, 'decision'));
    addpath(fullfile(projectRoot, 'decision', 'matrices'));
    addpath(fullfile(projectRoot, 'behavior'));
    addpath(fullfile(projectRoot, 'motion'));
    addpath(fullfile(projectRoot, 'mission'));
    addpath(fullfile(projectRoot, 'environment'));
    addpath(fullfile(projectRoot, 'utils'));
end
