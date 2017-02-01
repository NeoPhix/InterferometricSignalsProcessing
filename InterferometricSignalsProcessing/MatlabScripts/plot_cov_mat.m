signal = importdata('signal.txt');
restoredSignal = importdata('restoredSignal.txt');
CM = importdata('cov_mats.txt');
EKF = importdata('EKF_real_data.txt');

[N w] = size(EKF);

hFig = figure(1);
set(hFig, 'Position', [100 100 1600 800]);

subplot(2,3,1)
plot(1:N, signal(1:N));
grid on;
title('Signal');
xlabel('Discrete sample number');
ylabel('Signal value');
legend('Signal') ;

subplot(2,3,2)
plot(1:N, restoredSignal(1:N));
grid on;
title('Restored Signal');
xlabel('Discrete sample number');
ylabel('Signal value');
legend('Signal') ;

subplot(2,3,3)
plot(1:N, CM(1:N, 1));
grid on;
title('Background');
xlabel('Discrete sample number');
ylabel('Background value');
legend('EKF estimation') ;

subplot(2,3,4)
plot(1:N, CM(1:N, 5));
grid on;
title('Amplitude');
xlabel('Discrete sample number');
ylabel('Amplitude value');
legend('EKF estimation') ;

subplot(2,3,5)
plot(1:N, CM(1:N, 11));
grid on;
title('Frequency');
xlabel('Discrete sample number');
ylabel('Frequency value');
legend('EKF estimation') ;

subplot(2,3,6)
plot(1:N, CM(1:N, 16));
grid on;
title('Phase');
xlabel('Discrete sample number');
ylabel('Phase value, rad');
legend('EKF estimation') ;

