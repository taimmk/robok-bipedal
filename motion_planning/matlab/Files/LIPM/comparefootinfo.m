% Compare original and generated footinfos files
clear; clc; close all;

% 1. Load both datasets
if ~exist('defaultfootinfos_original.mat', 'file')
    error('Please rename your original backup file to "defaultfootinfos_backup.mat" before running this.');
end

original = load('defaultfootinfos_original.mat');
generated = load('defaultfootinfos.mat');

orig_foot = original.footinfos;
gen_foot = generated.footinfos;

% 2. Compare structural dimensions
fprintf('--- STRUCTURAL COMPARISON ---\n');
fprintf('Original steps:  %d\n', length(orig_foot));
fprintf('Generated steps: %d\n', length(gen_foot));

if length(orig_foot) ~= length(gen_foot)
    warning('The number of walking steps/phases does not match!');
end

% 3. Extract and reconstruct full continuous trajectories
t_orig = []; left_orig = []; right_orig = [];
t_gen = [];  left_gen = [];  right_gen = [];

% Reconstruct Original Trajectories
for i = 1:length(orig_foot)
    t_orig = [t_orig, orig_foot{i}.timevec];
    left_orig = [left_orig, orig_foot{i}.footleft([1 3 5], :)];
    right_orig = [right_orig, orig_foot{i}.footright([1 3 5], :)];
end

% Reconstruct Generated Trajectories
for i = 1:length(gen_foot)
    t_gen = [t_gen, gen_foot{i}.timevec];
    left_gen = [left_gen, gen_foot{i}.footleft([1 3 5], :)];
    right_gen = [right_gen, gen_foot{i}.footright([1 3 5], :)];
end

% 4. Compute Mean Squared Error (MSE) if sizes match
if length(t_orig) == length(t_gen)
    mse_left = mean((left_orig - left_gen).^2, 2);
    mse_right = mean((right_orig - right_gen).^2, 2);
    fprintf('\n--- TRAJECTORY ACCURACY (MSE) ---\n');
    fprintf('Left Foot MSE  -> X: %.6f, Y: %.6f, Z: %.6f\n', mse_left(1), mse_left(2), mse_left(3));
    fprintf('Right Foot MSE -> X: %.6f, Y: %.6f, Z: %.6f\n', mse_right(1), mse_right(2), mse_right(3));
else
    fprintf('\nNote: Cannot compute exact MSE because sequence lengths differ (Original: %d, Generated: %d).\n', ...
        length(t_orig), length(t_gen));
end

% 5. Plot Comparison (Left Foot)
figure('Name', 'Left Foot Trajectory Comparison', 'Units', 'Normalized', 'OuterPosition', [0.1 0.1 0.8 0.8]);

titles = {'X Position (Lateral)', 'Y Position (Forward)', 'Z Position (Vertical)'};
for dim = 1:3
    subplot(3, 1, dim);
    plot(t_orig, left_orig(dim, :), 'k-', 'LineWidth', 2.5); hold on;
    plot(t_gen, left_gen(dim, :), 'r--', 'LineWidth', 1.5);
    grid on;
    title(titles{dim});
    ylabel('Position (m)');
    if dim == 3, xlabel('Time (s)'); end
    legend('Original Backup', 'Generated New');
end