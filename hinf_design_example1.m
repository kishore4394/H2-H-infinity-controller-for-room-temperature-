clear all,close all
a = -10;
w0 = 700;

% Transfer function of generalized plant:
num11 = 10*sqrt(2)*w0; den11 = [1 w0];
num12 = [10*a*sqrt(2)*w0, -10*sqrt(2)*w0]; den12 = [1 w0+1 w0+1 w0];
num21 = 0; den21 = 1;
num22 = 0.001; den22 = 1;
num31 = 1; den31 = 1;
num32 = [a -1]; den32 = [1 1 1];

sys = tf({num11, num12; num21,num22; num31, num32}, ...
{den11,den12; den21,den22; den31, den32});

% Converting sys from transfer function to state space model
sys1 = ss(sys);

% Extracting A,B,C,D matrices from state space model
[A_in,B_in,C_in,D_in]=ssdata(sys1);

% Packing in the matrices for use by hinfsyn
p = pck(A_in,B_in,C_in,D_in);

[k,g,gfin]=hinfsyn(p,1,1,0,1,0.01);
% hinfsyn(NMEAS,NCONT,GAMMA_MIN,GAMMA_MAX,TOL)

% Unpacking the matrices of controller k
[Ak,Bk,Ck,Dk]=unpck(k);

% Converting the controller K to transfer function form
[numk,denk] = ss2tf(Ak,Bk,Ck,Dk);
sys_K = tf(numk,denk);

sys_P = tf([-a 1],[1 1 1]);
sys_KP = sys_K*sys_P;

% sys3 is the sensitivity transfer function (reference input r to error e)
sys3=1/(1+sys_KP);
% sys4 is the transfer function from reference input r to control effort 
sys4 = sys_K/(1+sys_KP);
figure(1),bodemag(sys3),grid
figure(2),bodemag(sys4),grid

% Transfer function of low pass filter
lp_f = tf(10*sqrt(2)*w0,[1 w0]);

% Transfer function from reference input r to z1
% Transfer function from reference input r to z2
sys_z1=lp_f - sys_P*lp_f*sys_K/(1+sys_P*sys_K);
sys_z2=0.001*sys_K/(1+sys_P*sys_K);

figure(3),bodemag(sys_z1),grid
figure(4),bodemag(sys_z2),grid

% Transfer function of closed loop system
sys_cl = [sys_z1;sys_z2];

% Verifying Hinfinity norm of sys_cl
norm(sys_cl,inf)

% a=10 (NMP zero), has w0=0.007;
% a=0.1 (NMP zero), has w0=0.5;
% a=0 (no zero), has w0=1.5;
% a=-0.1 (MP zero), has w0=7
% a=-10 (MP zero), has w0=700;