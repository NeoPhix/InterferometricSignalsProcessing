%%%
%'LOVE ANECHKA!!!'
%'WILL BACK AND GET "NA RUCHKI"!!!'
%%%
clear 
%Graphics
data = importdata('data.txt');
EKFdata = importdata('EKF_data.txt');
GDdata = importdata('GD_data.txt'); 
EKFGDdata = importdata('EKF_GD_data.txt'); 
[N w] = size(data);

hFig = figure(1);
set(hFig, 'Position', [100 100 1100 800]);

subplot(2,2,1)
plot(1:N, data(1:N, 2), 1:N, EKFdata(1:N, 2), 1:N, GDdata(1:N, 2), 1:N, EKFGDdata(1:N, 2));
grid on;
title('Amplitude');
xlabel('Discrete sample number');
ylabel('Amplitude value');
legend('Real amplitude','EKF estimation','GD estimation', 'EKF + GD estimation') ;

subplot(2,2,2)
plot(1:N, data(1:N, 3), 1:N, EKFdata(1:N, 3), 1:N, GDdata(1:N, 3), 1:N, EKFGDdata(1:N, 3));
grid on;
title('Frequency');
xlabel('Discrete sample number');
ylabel('Frequency value');
legend('Real amplitude','EKF estimation','GD estimation', 'EKF + GD estimation') ;

subplot(2,2,3)
plot(1:N, data(1:N, 4), 1:N, EKFdata(1:N, 4), 1:N, GDdata(1:N, 4), 1:N, EKFGDdata(1:N, 4));
grid on;
title('Phase');
xlabel('Discrete sample number');
ylabel('Phase value, rad');
legend('Real amplitude','EKF estimation','GD estimation', 'EKF + GD estimation') ;

zrs = 0.01*sin(1:N) ; %Заплатка, чтобы правильно цвета и подписи нарисовались на четвертом графике
subplot(2,2,4) 
plot(1:N, zrs, 1:N, data(1:N, 4) - EKFdata(1:N, 4),  1:N, data(1:N, 4) - GDdata(1:N, 4), 1:N, data(1:N, 4) - EKFGDdata(1:N, 4));
grid on;
title('Phase fail');
xlabel('Discrete sample number');
ylabel('Phase fail, rad');
legend('Real amplitude','EKF estimation','GD estimation', 'EKF + GD estimation') ;

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


