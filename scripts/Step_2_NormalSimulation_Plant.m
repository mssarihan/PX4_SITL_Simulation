%   Copyright 2020-2024 The MathWorks, Inc.

project = simulinkproject;
projectRoot = project.RootFolder;
open_system(fullfile(projectRoot,'models','Quad_UAV_Dynamics.slx'));