% Delete all variables, close all figures,  clear the workspace
clear all; close all; clc

%< -------------------- System matrices --------------------------->
A_1 = 2;                         % tank 1 cross sectional area, m^2
A_2 = 5;                         % tank 2 cross sectional area, m^2
R_1 = 1;                         % resistance to the flow F_1 
R_2 = 1;                         % resistance to the flow F_2
%h1  = 0;
%h2  = 1; 

sigma_x = [0.01, 0.01]';         % process noise std
sigma_y = 0.1;                   % measurement noise std

T = 0.001;                        % sample time
tend = 40;                       % simulation end time
t = 0:T:tend;                    % Time horizon for simulation
u_func   = @(t) sin(0.1*t)+1;    % Input function
u = u_func(t);
x0 = [0 1]';                     % True Initial state of the process 

A = [(-R_1/A_1) 0;(R_1/A_2) (-R_2/A_2)];        % State matrix
B = [1/A_1; 0];                                 % Input matrix
C = [0 1];                                      % Output matrix
D = 0;
W = eye(size(A));
V = [0 1];%1


%< ----------------- Simulate the system -------------------------->
[t,h1,h2]=CascadeSimulator(A,B,x0,sigma_x,sigma_y,tend,u_func,T);
yn = h2;

%< ---------Kalman Filter for the Extended System ----------------->
Bd = T*B;% Discrete Input matrix
W = T*W;

x0_hat = [0.5 0.5]';                    % Estimate (guess) of initial state
P0     = [0.01 0; 0 0.01];              % Covariance of initial state estimate
Q      = [0.1 0; 0 0.1];                % State noise covariance matrix
R      = 1;                             % Measurement noise covariance matirx

xhat(:,1) = x0_hat;
P = P0;
Pout{1,1} = P0;

for i=2:length(yn)
    % time update
    Ad = [(-R_1*T/(2*A_1*sqrt(xhat(1,i-1))))+1                 0                   ;...
           R_1*T/(2*A_1*sqrt(xhat(1,i-1)))    (-R_2*T/(2*A_2*sqrt(xhat(2,i-1))))+1];
    xhat(:,i) = xhat(:,i-1) + T*A* sqrt(xhat(:,i-1)) + Bd*u(i-1);
    P =  Ad * P * Ad' + (W*Q*W'); 
    
    % Measurement update
    K =  P * C'*(C*P*C'+V*R*V')^(-1);
    xhat(:,i) =  xhat(:,i) + K*(yn(:, i)' - C*xhat(:,i));%-h(x,u,0))
    P = (eye(size(P)) - K*C)*P*(eye(size(P)) - K*C)'+ K*V*R*V'*K';
    Pout{i,1} = P;
end

%< ------------------------ Plot the results ----------------------->
figure()
subplot(1,2,1)
plot(t,h1(1,:),'LineWidth', 3)
hold on
plot(t,xhat(1,:), '-.r','LineWidth',3)
grid on
set(gca,'FontSize',20)
xlabel('Time [s] ', 'FontSize',20)
ylabel('h1 [m]','FontSize',20)

subplot(1,2,2)
plot(t,h2(1,:),'LineWidth', 3)
hold on
plot(t,xhat(2,:), '-.r','LineWidth',3)
grid on
set(gca,'FontSize',20)
xlabel('Time [s]','FontSize',20)
ylabel('h2 [m]','FontSize',20)

legend('Process', 'Kalman Estimate')
%< ----------------------------------------------------------------->
