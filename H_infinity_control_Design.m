clear all;
close all;
clc;

% applying pade function to eliminate time delay
num =[1]; den = [1 1 2]; sys = tf(num,den,'InputDelay',1);
sysx = pade(sys,1);

a = 100;
w0 = 50;

%calculating the plant matrix 
num11 = 100*sqrt(2)*w0; den11 = [1 w0];
num12 = [100*sqrt(2)*w0 -200*sqrt(2)*w0]; den12 = [1 3+w0 4+3*w0 4+4*w0 4*w0];
num21 = 0; den21 = 1;
num22 = 0.01; den22 = 1;
num31 = 1; den31 = 1;
num32 = [1 -2]; den32 = [1 3 4 4];

sys = tf({num11, num12; num21,num22; num31, num32}, ...
{den11,den12; den21,den22; den31, den32});

%finding the state space model and the state matrices
sys1 = ss(sys);
[A_in,B_in,C_in,D_in] = ssdata(sys1);

%packing in the matrices for finding H_infinity of the system
p = pck(A_in,B_in,C_in,D_in);
[k,g,gfin] = hinfsyn(p,1,1,0,1000,0.01);

%unpacking it to find the gain transfer function
[Ak,Bk,Ck,Dk] = unpck(k);
[numk,denk]=ss2tf(Ak,Bk,Ck,Dk);
sys_k = tf(numk,denk);


sys_g = tf([-1 2],[1 3 4 4]);
sys_gk = sys_k*sys_g;

%finding the transfer function for sensitivity
sys3 = 1/(1+sys_gk);

%finding the transfer function for complementary sensitivity
sys4 = sys_k/(1+sys_gk);

%plot the bode plot of the two transfer functions
figure(7),bodemag(sys3),grid
figure(8),bodemag(sys4),grid

%transfer function of low pass filter
lp_f = tf(100*sqrt(2)*w0,[1 w0]);

sys_z1=lp_f - sys_g*lp_f*sys_k/(1+sys_g*sys_k);
sys_z2=0.01*sys_k/(1+sys_g*sys_k);

figure(9),bodemag(sys_z1),grid
figure(10),bodemag(sys_z2),grid

%closed loop transfer function
sys_cl = [sys_z1;sys_z2];

%H_infinity norm of the closed loop system
norm(sys_cl,inf)
