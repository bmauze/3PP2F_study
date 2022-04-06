%% Import data from text file
% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 52);
% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = [",", "]["];
% Specify column names and types
opts.VariableNames = ["VarName1", "VarName2", "VarName3", "VarName4", "VarName5", "VarName6", "VarName7", "VarName8", "VarName9", "VarName10", "VarName11", "VarName12", "VarName13", "VarName14", "VarName15", "VarName16", "VarName17", "VarName18", "VarName19", "VarName20", "VarName21", "e05", "VarName23", "VarName24", "VarName25", "VarName26", "VarName27", "e08", "e09", "VarName30", "VarName31", "VarName32", "VarName33", "e10", "e09_1", "VarName36", "VarName37", "VarName38", "VarName39", "VarName40", "VarName41", "VarName42", "e05_1", "e05_2", "e06", "VarName46", "e05_3", "VarName48", "VarName49", "VarName50", "VarName51", "VarName52"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";
% Specify variable properties
opts = setvaropts(opts, ["VarName1", "VarName52"], "TrimNonNumeric", true);
opts = setvaropts(opts, ["VarName1", "VarName52"], "ThousandsSeparator", ",");
%% Import and range the data of the Circle Sim
SimRescircle = readtable("G:\Utilisateurs\benjamin.mauze\Nextcloud\SOFA_Current_work\Design_without_constraints\Travail_forum\3PP2F\SimRes_circle.txt", opts);
% Convert to output type
SimRescircle = table2array(SimRescircle);
% Ranging variables for Circle sim
q1PosSimCir=SimRescircle(:,1:7)-SimRescircle(1,1:7); q2PosSimCir=SimRescircle(:,8:14)-SimRescircle(1,8:14); q3PosSimCir=SimRescircle(:,15:21)-SimRescircle(1,15:21);
q1ForceSimCir=SimRescircle(:,22:27); q2ForceSimCir=SimRescircle(:,28:33); q3ForceSimCir=SimRescircle(:,34:39);
posPltSimCir=SimRescircle(:,40:46); forcePltSimCir=SimRescircle(:,47:end); 
QposSimCir=[q1PosSimCir(:,1:2),q2PosSimCir(:,1:2),q3PosSimCir(:,1:2)];

dlmwrite('QposSimCir.txt',QposSimCir);
RobotOffset=[0.0, 75.00,-64.95,-37.496,64.950,-37.498];
QpSmCrRr=RobotOffset + QposSimCir;
dlmwrite('QpSmCrRr.txt',QpSmCrRr);
%% Import and range the data of the Square Sim
SimResSqr = readtable("G:\Utilisateurs\benjamin.mauze\Nextcloud\SOFA_Current_work\Design_without_constraints\Travail_forum\3PP2F\SimRes_square.txt", opts);
% Convert to output type
SimResSqr = table2array(SimResSqr);
% Ranging variables for square sim
q1PosSimSqr=SimResSqr(:,1:7)-SimResSqr(1,1:7); q2PosSimSqr=SimResSqr(:,8:14)-SimResSqr(1,8:14); q3PosSimSqr=SimResSqr(:,15:21)-SimResSqr(1,15:21);
q1ForceSimSqr=SimResSqr(:,22:27); q2ForceSimSqr=SimResSqr(:,28:33); q3ForceSimSqr=SimResSqr(:,34:39);
posPltSimSqr=SimResSqr(:,40:46); forcePltSimSqr=SimResSqr(:,47:end); 
QposSimSqr=[q1PosSimSqr(:,1:2),q2PosSimSqr(:,1:2),q3PosSimSqr(:,1:2)];

dlmwrite('QposSimSqr.txt',QposSimSqr);
QpSmSqRr=RobotOffset + QposSimSqr;
dlmwrite('QpSmSqRr.txt',QpSmSqRr);
%% Clear temporary variables
clear opts
