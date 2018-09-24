% Define a first order transfer function 1/(s+1)

num = 1; den = [1 1];
sys1 = tf(num,den);

% Define a second order transfer function (s+1)/(s^2 + 3s + 2)

num = [1 1]; den = [1 3 2];
sys2 = tf(num,den);

% Define a first order transfer function with delay of 0.3 sec:
% e^(-0.3s)10/(s+2)

num = 10; den = [1 2];
sys3 = tf(num,den,'InputDelay',0.3);

% Draw a Bode plot for system 1
bode(sys1),grid

% Simulate a system
tspan = [0:0.1:10];
u = ones(length(tspan),1);
% Simulating a first order system
% Can use lsim only when we are simulating a linear system
% For simulating nonlinear systems, use ode23 or ode45, etc.
lsim(sys1,u,tspan)

% Simulating a first order system with a time delay
lsim(sys3,u,tspan)

% Simulating a second order system
lsim(sys2,u,tspan)

% Simulating a second order system with smaller damping

num = [1 1]; den = [1 1 2];
sys4 = tf(num,den);
lsim(sys4,u,tspan)
bode(sys4)
