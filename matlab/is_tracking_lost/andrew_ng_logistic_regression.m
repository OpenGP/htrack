function [f, theta] = andrew_ng_logistic_regression(x, y)

[m, n] = size(x);

%% Add intercept term to x
x = [ones(m, 1), x]; 

%% Initialize fitting parameters
theta = zeros(n+1, 1);

%% Define the sigmoid function
g = @(z) 1.0 ./ (1.0 + exp(-z)); 

%% Newton's method
MAX_ITR = 7;
%f = zeros(MAX_ITR, 1);

for i = 1:MAX_ITR
    %% Calculate the hypothesis function
    z = x * theta;
    h = g(z);
    
    %% Calculate gradient and hessian.
    grad = (1/m).*x' * (h-y);
    H = (1/m).*x' * diag(h) * diag(1-h) * x;           
    theta = theta - H\grad;    
end

%% Calculate objective function and error
h = g(x * theta);
f = mean(abs(h - y));



