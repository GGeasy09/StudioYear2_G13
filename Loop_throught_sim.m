clear;
folderPath = 'C:\Users\ACER\Documents\GitHub\StudioYear2_G13\Lab1 Part2 Parameter';   % ← change this
files = dir(fullfile(folderPath, '*.mat'));

for i = 1:length(files)
    filename = fullfile(folderPath, files(i).name);
    load(filename);
    sim("C:\Users\ACER\Documents\GitHub\StudioYear2_G13\Lab1_parameter_estimation_student.slx");
    Sample_Signal = ans.Input_Signal.signals.values;
    Sample_time = ans.tout;
      figure;
      plot(Sample_time, Sample_Signal);
xlabel('Time (s)');
ylabel('Signal Amplitude');
title(filename);
grid on;

    save(filename);
end

