%%%
%'LOVE ANECHKA!!!'
%'WILL BACK AND GET "NA RUCHKI"!!!'
%%%

clear;
%Graphics
data = importdata('GD_1_deviations.txt');
EKFdata = importdata('GD_1_deviations.txt');
GDdata = importdata('GD_1_deviations.txt');

[N w] = size(data);

hFig = figure(1);
set(hFig, 'Position', [100 100 1100 800]);

subplot(2,2,1)
plot(1:N, data(1:N, 1), 1:N, EKFdata(1:N, 1), 1:N, GDdata(1:N, 1));
grid on;
title('Amplitude');
xlabel('Discrete sample number');
ylabel('Amplitude value');
legend('Real amplitude','EKF estimation','EKF after gradient descent') ;

subplot(2,2,2)
plot(1:N, data(1:N, 2), 1:N, EKFdata(1:N, 2), 1:N, GDdata(1:N, 2));
grid on;
title('Amplitude');
xlabel('Discrete sample number');
ylabel('Amplitude value');
legend('Real amplitude','EKF estimation','EKF after gradient descent') ;

subplot(2,2,3)
plot(1:N, data(1:N, 3), 1:N, EKFdata(1:N, 3),  1:N, GDdata(1:N, 3));
grid on;
title('Frequency');
xlabel('Discrete sample number');
ylabel('Frequency value');
legend('Real frequency','EKF estimation','EKF after gradient descent') ;

subplot(2,2,4)
plot(1:N, data(1:N, 4), 1:N, EKFdata(1:N, 4),  1:N, GDdata(1:N, 4));
grid on;
title('Phase');
xlabel('Discrete sample number');
ylabel('Phase value, rad');
legend('Real phase','EKF estimation','EKF after gradient descent') ;

%Background
% figure, plot(1:N, background, 1:N, outParticle(1, 1:N), 1:N, outParticleKalman(1, 1:N), 1:N, outKalman(1, 1:N));
% grid on;
% title('Background');
% xlabel('Discrete sample number');
% ylabel('Background value');
% legend('Real Background','Particle filter estimation','EKPF estimation','Kalman estimation') ;

std(EKFdata(1:N, 1) - data(1:N, 1));

estimationError = zeros(5, 4) ;
estimationError(1,1) = std(EKFdata(1:N, 1) - data(1:N, 1));
estimationError(1,2) = std(EKFdata(1:N, 2) - data(1:N, 2));
estimationError(1,3) = std(EKFdata(1:N, 3) - data(1:N, 3));
estimationError(1,3) = std(EKFdata(1:N, 4) - data(1:N, 4));


%%
% for i = 1:N
%     outKalman(5,i) = outKalman(1,i) + outKalman(2,i)*cos((outKalman(4,i)));
%     outParticle(5,i) = outParticle(1,i) + outParticle(2,i)*cos((outParticle(4,i)));
%     outParticleKalman(5,i) = outParticleKalman(1,i) + outParticleKalman(2,i)*cos((outParticleKalman(4,i)));
% end
% estimationError(5,1) = 20*log10(std(corruptedSignal)/std(noise)) ;
% estimationError(5,2) = 20*log10(std(outKalman(5, 1:N))/std(corruptedSignal - outKalman(5, 1:N))) ;
% estimationError(5,3) = 20*log10(std(outParticle(5, 1:N))/std(corruptedSignal - outParticle(5, 1:N))) ;
% estimationError(5,4) = 20*log10(std(outParticleKalman(5, 1:N))/std(corruptedSignal - outParticleKalman(5, 1:N))) ;
% 


