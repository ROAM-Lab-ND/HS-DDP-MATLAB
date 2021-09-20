function compare_hsddp2saltation(hsddp_data)
folderName = 'BouncingBall/Data/Nathan/';
fileName = 'saltation_matrix_trajectory_1_bounce.mat';
u_struct = load(strcat(folderName, fileName),'u_trj');
x_struct = load(strcat(folderName, fileName),'x_trj');
K_struct = load(strcat(folderName, fileName),'K_trj');
u_sal = u_struct.u_trj;
x_sal = x_struct.x_trj;
K_sal = K_struct.K_trj;

x_hsddp = [];
u_hsddp = [];
for i = 1:length(hsddp_data.xopt)
    x_traj_i = cell2mat(hsddp_data.xopt{i});
    len = length(hsddp_data.xopt{i});
    if i < length(hsddp_data.xopt)
        len = length(hsddp_data.xopt{i}) - 1;
    end
    x_hsddp = [x_hsddp, x_traj_i(:,1:len)];
    u_hsddp = [u_hsddp,cell2mat(hsddp_data.uopt{i})];
end
x_hsddp = x_hsddp';
u_hsddp = u_hsddp';
Kopt = hsddp_data.Kopt;
Kopt{1}(end) = [];
Kopt{2}(end) = [];
Kopt = cat(2,Kopt{:});
K_hsddp = cell2mat(Kopt(:));
K_sal = cell2mat(K_sal(:));

%% 
t = (0:999)*hsddp_data.dt;
figure
subplot(3,1,1)
plot(t, x_sal(:,1));
hold on
plot(t, x_hsddp(:,1),'--');
xlabel('time (s)');
ylabel('z (m)');
legend('Saltation', 'HSDDP');

subplot(3,1,2)
plot(t, x_sal(:,2));
hold on;
plot(t, x_hsddp(:,2),'--');
xlabel('time (s)');
ylabel('zd (m/s)');

subplot(3,1,3)
plot(t(1:end-1), u_sal);
hold on;
plot(t(1:end-1), u_hsddp,'--');
xlabel('time (s)');
ylabel('u (N)');

figure
subplot(2,1,1)
plot(t(1:end-1), K_sal(:,1));
hold on
plot(t(1:end-1), K_hsddp(:,1),'--');
xlabel('time (s)');
ylabel('K1')
subplot(2,1,2)
plot(t(1:end-1), K_sal(:,2));
hold on
plot(t(1:end-1), K_hsddp(:,2),'--');
xlabel('time (s)');
ylabel('K2');
legend('Saltation', 'HSDDP');

end