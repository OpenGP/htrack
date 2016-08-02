clear; close all; clc;
num_frames = 5500;

%% Create labels
success_and_failure_switch = [1, 8, 2240, 2321, 2423, 2586, 2685, 3082, 3128, 3189, 3231];
success_and_failure_switch = [1, success_and_failure_switch, num_frames];
Labels = ones(num_frames, 1);
for i = 1:length(success_and_failure_switch) - 1
    if (rem(i, 2) == 1)
        Labels(success_and_failure_switch(i):success_and_failure_switch(i + 1)) = 0;
    else
        Labels(success_and_failure_switch(i):success_and_failure_switch(i + 1)) = 1;
    end    
end

%% Load c++ data
fileID = fopen('data.txt','r');
Data = fscanf(fileID, '%f %f');
fclose(fileID);
Data = [Data(1:2:end) Data(2:2:end)];

%% Process the data
I1 = Data(:, 1) < 10e16;
I2 = Data(:, 2) < 10e16;
I = (I1 .* I2) == 1;
Data = Data(I, :);
Labels = Labels(I, :);
x = Data;
y = Labels;

%% Train the classifier
[error, theta] = andrew_ng_logistic_regression(x, y);

%% Plot the training data
figure; hold on;
success_indices = find(y == 1); 
failure_indices = find(y == 0);
scatter(x(failure_indices, 1), x(failure_indices, 2), 7, [0.9, 0.1, 0.8], 'filled')
scatter(x(success_indices, 1), x(success_indices, 2), 7, [0, 0.7 0.7], 'filled')
xlabel('Intersection / Union')
ylabel('Average Distance')

%% Plot the decision boundary line
%theta(1) = -2; - you can manually pick the offset, to ensure that the
%classifier gives false positives less often;
alpha = 0.5;
plot_x = [0, 1];
plot_y = (-1./theta(3)).*(theta(2).*plot_x +theta(1) + log(1/alpha - 1)) + 1;
plot(plot_x, plot_y, 'LineWidth', 2);
legend('Success', 'Failure', 'Boundary')
ylim([3, 15]);
set(gca,'fontsize', 12);



