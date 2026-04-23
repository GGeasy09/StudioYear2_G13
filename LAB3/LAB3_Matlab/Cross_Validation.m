clear; close all;
load("LAB3_Matlab_Result\Graph\EXP1 Velo3 1_2m 5Nut.mat");
data_result_1 = data.getElement("Offset_V3");
load("LAB3_Matlab_Result\Estimate Result\EXP1.mat");
Experiment_Value = SDOSessionData.Data.Workspace.LocalWorkspace.Exp.OutputData.Values.value;
Experiment_Time = SDOSessionData.Data.Workspace.LocalWorkspace.Exp.OutputData.Values.Time;

% --- Default Inputs ---
g = 9.81;
k = 29.415;
b = 0;
m_def = 0.115;
mass_0 = 0.149;
init_pos_def = 0.0978;
offset_time_def = 0.3;