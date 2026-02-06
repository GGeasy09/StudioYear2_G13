% Perform Fourier Transform
Fs = 1 / mean(diff(time)); % Sampling frequency
N = length(speed); % Number of samples
Y = fft(speed); % Compute the Fourier Transform
f = (0:N-1)*(Fs/N); % Frequency range
P2 = abs(Y/N); % Two-sided spectrum
P1 = P2(1:N/2+1); % Single-sided spectrum
P1(2:end-1) = 2*P1(2:end-1); % Correct amplitude

% Plot the results
figure;
plot(f(1:N/2+1), P1);
title('Single-Sided Amplitude Spectrum of Speed');
xlabel('Frequency (f)');
ylabel('|P1(f)|');
grid on;
