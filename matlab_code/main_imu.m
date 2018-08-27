close all;clear all;clc;
addpath('geometry_library');
%%

data = importdata('../save_exp/imu.txt');
data.data(1,:) = [];

t_imu = data.data(:,1); t_imu = t_imu - t_imu(1); acc_imu = data.data(:,2:4); 
w_imu = data.data(:,5:7).'; mag_imu = data.data(:,8:10).';
q_imu = data.data(:,11:14).'; q_imu = q_imu([2,3,4,1],:);

q_imu_init = q_imu(:,1);

for i = 1:length(q_imu)
   q_imu(:,i) = quat_prod_kch(q_imu(:,i), quat_inv_kch(q_imu_init));  
end

%% Magnetometer
% figure();
% subplot(3,1,1); plot(t_imu, mag_imu(1,:)); ylabel('mag x [mG]'); ylim([-1,1]*0.5);
% subplot(3,1,2); plot(t_imu, mag_imu(2,:)); ylabel('mag y [mG]'); ylim([-1,1]*0.5);
% subplot(3,1,3); plot(t_imu, mag_imu(3,:)); ylabel('mag z [mG]'); xlabel('time [s]'); ylim([-1,1]*0.5);
% 
% figure();
% plot(mag_imu(1,:), mag_imu(2,:)); xlabel('mag x'); ylabel('mag y'); hold on;
% plot(0,0,'r+','linewidth',2);
% axis equal; xlim([-1,1]*0.5); ylim([-1,1]*0.5); 
% 

%% Accelerometer
figure();
subplot(3,1,1); plot(t_imu, acc_imu(:,1)); ylabel('acc x [m/s^2]'); ylim([-1,1]*9.81*2);
subplot(3,1,2); plot(t_imu, acc_imu(:,2)); ylabel('acc y [m/s^2]'); ylim([-1,1]*9.81*2);
subplot(3,1,3); plot(t_imu, acc_imu(:,3)); ylabel('acc z [m/s^2]'); ylim([-1,1]*9.81*2);

fprintf('acc x-m: %0.4f, std: %0.4f // y-m: %0.4f, std: %0.4f // z-m: %0.4f, std: %0.4f\n',mean(acc_imu(:,1)),std(acc_imu(:,1)),mean(acc_imu(:,2)),std(acc_imu(:,2)),mean(acc_imu(:,3)),std(acc_imu(:,3)));

%% Gyro
bias_w = [mean(w_imu(1,1:300)),mean(w_imu(2,1:300)),mean(w_imu(3,1:300))].';
figure();
subplot(3,1,1); plot(t_imu, w_imu(1,:)-bias_w(1,1)); ylabel('w_x [rad/s]'); ylim([-1,1]*5);
subplot(3,1,2); plot(t_imu, w_imu(2,:)-bias_w(2,1)); ylabel('w_y [rad/s]'); ylim([-1,1]*5);
subplot(3,1,3); plot(t_imu, w_imu(3,:)-bias_w(3,1)); ylabel('w_z [rad/s]'); xlabel('time [s]'); ylim([-1,1]*5);


%% Integration
q_int = zeros(4,length(t_imu));
q_int_RK = zeros(4,length(t_imu));

norm_q = zeros(1,length(t_imu));
norm_q(1)=1;
q_int(:,1) = [1;0;0;0];
q_int_RK(:,1) = [1;0;0;0];

for i=1:length(t_imu)-1
    dt = t_imu(i+1) - t_imu(i);
    w_temp = w_imu(:,i) - bias_w;

    q_int(:,i+1) = q_int(:,i) + dt*quat_derivative_kch(q_int(:,i),w_temp);
    q_int(:,i+1) = q_int(:,i+1)/norm(q_int(:,i+1));
    norm_q(i+1) = norm(q_int(:,i));
end

% Runge-Kutta 4th order integration
for i=1:length(t_imu)-1
   dt = t_imu(i+1) - t_imu(i);
   w_temp = w_imu(:,i) - bias_w;
%    w_temp = -w_temp;
   
   k1 = quat_derivative_kch(q_int_RK(:,i), w_temp);
   k2 = quat_derivative_kch(q_int_RK(:,i)+k1*dt/2, w_temp);
   k3 = quat_derivative_kch(q_int_RK(:,i)+k2*dt/2, w_temp);
   k4 = quat_derivative_kch(q_int_RK(:,i)+k3*dt,   w_temp);
   
   q_int_RK(:,i+1) = q_int_RK(:,i) + dt/6*(k1 + 2*k2 + 2*k3 + k4);
   
   q_int_RK(:,i+1) = q_int_RK(:,i+1)/norm(q_int_RK(:,i+1));
end

figure();
subplot(4,1,1); plot(t_imu, q_imu(1,:),'k'); hold on; plot(t_imu(1:end), q_int(1,:),'b'); plot(t_imu, q_int_RK(1,:),'r'); legend('int','truth'); ylim([-1.5,1.5]);
subplot(4,1,2); plot(t_imu, q_imu(2,:),'k'); hold on; plot(t_imu(1:end), q_int(2,:),'b'); plot(t_imu, q_int_RK(2,:),'r'); ylim([-1.5,1.5]);
subplot(4,1,3); plot(t_imu, q_imu(3,:),'k'); hold on; plot(t_imu(1:end), q_int(3,:),'b'); plot(t_imu, q_int_RK(3,:),'r'); ylim([-1.5,1.5]);
subplot(4,1,4); plot(t_imu, q_imu(4,:),'k'); hold on; plot(t_imu(1:end), q_int(4,:),'b'); plot(t_imu, q_int_RK(4,:),'r'); ylim([-1.5,1.5]);

%% Euler angle omparison

E_int = quat_to_euler(q_int);
E_imu = quat_to_euler(q_imu);
E_int_RK = quat_to_euler(q_int_RK);


figure();
subplot(3,1,1); plot(t_imu,E_int(1,:)/pi*180); hold on; plot(t_imu, E_imu(1,:)/pi*180); plot(t_imu, E_int_RK(1,:)/pi*180); ylabel('\phi [deg]'); legend('int','imu','int RK'); ylim([-1,1]*180);
subplot(3,1,2); plot(t_imu,E_int(2,:)/pi*180); hold on; plot(t_imu, E_imu(2,:)/pi*180); plot(t_imu, E_int_RK(2,:)/pi*180); ylabel('\theta [deg]'); ylim([-1,1]*180);
subplot(3,1,3); plot(t_imu,E_int(3,:)/pi*180); hold on; plot(t_imu, E_imu(3,:)/pi*180); plot(t_imu, E_int_RK(3,:)/pi*180); ylabel('\psi [deg]');  ylim([-1,1]*180);

