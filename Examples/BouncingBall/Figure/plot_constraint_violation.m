function plot_constraint_violation(vlts)
figure
plot(0:length(vlts)-1, vlts);
xlabel('Outer-loop iteration');
ylabel('Maximum violation (m)');
end