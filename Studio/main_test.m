%Transfer Function for a passive, first-order, high-pass filter: 1/wRC
x = linspace(0.01,1,100); 
%For this example, let's set R = 1 kOhm and C = 1 uF
%The variable x represents angular frequency w
R = 1000;
C = 0.000001;
g = (sqrt(R.^2 * x.^2 * C.^2))./(sqrt(1 + (R.^2 * x.^2 * C.^2)));
y = -20*log10(g);
plot(x,y);
grid on
% % xlim([0 0.05]);
% % ylim([0 0.05]);
% set(gca, 'XScale','log')                                        % Optional