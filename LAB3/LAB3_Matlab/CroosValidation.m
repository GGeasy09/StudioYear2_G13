% =========================================================================
% Script: Cross-Validation สำหรับระบบ Mass-Spring-Damper
% รันการจำลอง 36 รูปแบบ (6 Experiments x 6 Parameter Sets) ด้วยคำสั่ง parsim
% =========================================================================
clear; clc; close all;

%% 1. กำหนดค่าคงที่และชุดพารามิเตอร์เริ่มต้น
g = 9.81;
mass_0 = 0.148;
mdl = 'untitled'; % ชื่อไฟล์โมเดล Simulink ของคุณ

% โหลดโมเดล Simulink ขึ้นมาเตรียมไว้ในหน่วยความจำ
load_system(mdl);

% ชุดพารามิเตอร์ [k, b] ทั้ง 6 ชุดที่ต้องการนำมาเทียบ
k_sets = [29.87588543, 29.254, 29.351, 29.94, 31.389, 29.501];
b_sets = [0.04726935407, 0.038389, 0.06387, 0.056827, 0.13077, 0.049731];
num_param_sets = length(k_sets);

% ข้อมูลมวลและตำแหน่งเริ่มต้นของแต่ละ EXP (1-6)
m_def_exp = [0.191, 0.114, 0.114, 0.191, 0.038, 0.038];
init_pos_exp = [0.1295, 0.098, 0.0776, 0.1041, 0.044, 0.030];
num_exps = length(m_def_exp);

% สร้างเมทริกซ์ 6x6 เพื่อเก็บค่าความคลาดเคลื่อน (RMSE)
RMSE_matrix = zeros(num_exps, num_param_sets);

%% 2. เริ่มทำงาน (ลูปทีละ 1 การทดลอง เพื่อเทียบกับพารามิเตอร์ 6 ชุด)
for exp_idx = 1:num_exps
    fprintf('กำลังโหลดข้อมูลและจำลองระบบสำหรับ EXP%d...\n', exp_idx);
    
    % --- 2.1 โหลดข้อมูลทดลองอ้างอิง (Input Graph) ---
    fileName = sprintf('LAB3_Matlab_Result_Estimate\\EXP%d.mat', exp_idx);
    
    % ตรวจสอบว่าไฟล์มีอยู่จริง หากไม่มีให้ข้ามไปการทดลองถัดไป
    if ~isfile(fileName)
        warning('ไม่พบไฟล์ %s ขอข้ามการคำนวณนี้', fileName);
        RMSE_matrix(exp_idx, :) = NaN;
        continue;
    end
    
    % โหลดข้อมูลจากไฟล์ .mat เข้า Workspace ชั่วคราว
    loadedData = load(fileName);
    % เข้าถึง Data ตามเส้นทางโครงสร้างที่คุณให้มา
    expDataObj = loadedData.SDOSessionData.Data.Workspace.LocalWorkspace.Exp.OutputData.Values;
    
    % ดึงแกนเวลาและแกนข้อมูล
    expTime = expDataObj.Time;
    expPos = expDataObj.Data;
    
    % กำหนดระยะเวลาการจำลอง (Stop Time) ให้เท่ากับวินาทีสุดท้ายของข้อมูลจริง
    stopTime = expTime(end);
    
    % --- 2.2 จัดเตรียมข้อมูล (SimulationInput) สำหรับรันพร้อมกัน 6 ค่า ---
    % เคลียร์ตัวแปร in ให้ว่าง และจองพื้นที่สำหรับ 6 ชุดข้อมูล
    in = Simulink.SimulationInput.empty(0, num_param_sets);
    
    for p_idx = 1:num_param_sets
        in(p_idx) = Simulink.SimulationInput(mdl);
        
        % กำหนดค่าคงที่
        in(p_idx) = in(p_idx).setVariable('g', g);
        in(p_idx) = in(p_idx).setVariable('mass_0', mass_0);
        
        % กำหนดตัวแปรมวล (ส่งไปทั้งชื่อ m และ m_def เผื่อบล็อกข้างในเรียกใช้ชื่อต่างกัน)
        in(p_idx) = in(p_idx).setVariable('m', m_def_exp(exp_idx)); 
        in(p_idx) = in(p_idx).setVariable('m_def', m_def_exp(exp_idx));
        
        % กำหนดตัวแปรตำแหน่งเริ่มต้น 
        in(p_idx) = in(p_idx).setVariable('init_pos_def', init_pos_exp(exp_idx));
        in(p_idx) = in(p_idx).setVariable('Initial_Point', init_pos_exp(exp_idx));
        
        % กำหนดค่า [k, b] ที่ต้องการทดสอบ
        in(p_idx) = in(p_idx).setVariable('k', k_sets(p_idx));
        in(p_idx) = in(p_idx).setVariable('b', b_sets(p_idx));
        
        % ตั้งค่าเวลาสิ้นสุดการจำลอง
        in(p_idx) = in(p_idx).setModelParameter('StopTime', num2str(stopTime));
    end
    
    % --- 2.3 รัน Simulation แบบ Parallel (ขนานกัน 6 ตัวประหยัดเวลา) ---
    out = parsim(in, 'ShowProgress', 'off');
    
    % --- 2.4 นำผลลัพธ์จาก Simulink มาเทียบกับข้อมูลจริง (หา RMSE) ---
    for p_idx = 1:num_param_sets
        % ป้องกัน Error กรณี Simulink รันไม่สำเร็จ
        if ~isempty(out(p_idx).ErrorMessage)
            warning('เกิดข้อผิดพลาดในการรัน EXP%d รูปแบบที่ %d: %s', exp_idx, p_idx, out(p_idx).ErrorMessage);
            RMSE_matrix(exp_idx, p_idx) = NaN;
            continue;
        end
        
        % ดึงข้อมูลจากการบันทึก (สัญลักษณ์จุดรับสัญญาณสีฟ้าบนเส้น x ในโมเดล)
        simLoggedData = out(p_idx).logsout.getElement(1).Values; 
        simTime = simLoggedData.Time;
        simPos = simLoggedData.Data;
        
        % Interpolation: ข้อมูล Simulink กับข้อมูลจริงอาจสุ่มตัวอย่าง (Sample time) 
        % ไม่ตรงกัน จึงต้องปรับจุดกราฟ Simulink ให้ตรงกับเวลาของข้อมูลจริงก่อนเทียบ
        simPos_interp = interp1(simTime, simPos, expTime, 'linear', 'extrap');
        
        % คำนวณ Root Mean Square Error (RMSE)
        error = expPos - simPos_interp;
        RMSE_matrix(exp_idx, p_idx) = sqrt(mean(error.^2));
    end
end

%% 3. สรุปผลลัพธ์ (Display Results)
disp('-----------------------------------------------------------');
disp('      ตารางผลลัพธ์ RMSE (Root Mean Square Error)            ');
disp('-----------------------------------------------------------');

% นำเมทริกซ์ 6x6 มาแสดงผลในรูปแบบตารางให้อ่านง่าย
k_labels = arrayfun(@(x) sprintf('Set %d', x), 1:6, 'UniformOutput', false);
exp_labels = arrayfun(@(x) sprintf('EXP %d', x), 1:6, 'UniformOutput', false);
RMSE_Table = array2table(RMSE_matrix, 'VariableNames', k_labels, 'RowNames', exp_labels);
disp(RMSE_Table);

% นำค่า RMSE ของแต่ละคอลัมน์ (ชุด k,b แต่ละชุด) มารวมกันเพื่อหาโมเดลที่ครอบคลุมมากที่สุด
total_RMSE_per_set = sum(RMSE_matrix, 1);
[min_total_RMSE, best_idx] = min(total_RMSE_per_set);

fprintf('\n===========================================================\n');
fprintf('>>> ชุดพารามิเตอร์ที่ดีที่สุดสำหรับการทดลองทั้งหมดคือ: ชุดที่ %d <<<\n', best_idx);
fprintf('k = %f, b = %f\n', k_sets(best_idx), b_sets(best_idx));
fprintf('มีผลรวมความคลาดเคลื่อน (Total RMSE) ต่ำสุดที่: %f\n', min_total_RMSE);
fprintf('===========================================================\n');

% ปิดโมเดล
bdclose(mdl);