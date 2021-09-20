function initializeBouncingTraj(hybridT)
folderName = '/home/wensinglab/HL/Code/HSDDP/MATLAB/Examples/BouncingBall/Data/Nathan/';
fileName = 'saltation_matrix_trajectory_1_bounce.mat';
u_struct = load(strcat(folderName, fileName),'u_trj');
u = u_struct.u_trj;
hybridT(1).Ubar = num2cell(u(1:543));
hybirdT(2).Ubar = num2cell(u(544:end));
end