clear all
close all
clc

L = 0.01;
M = 0.02;
N = -0.03;
Ixx = 1;
Iyy = 1.1;
Izz = 0.9;

tstep = 0.001;
dt = tstep;
tf = 50;
t = 0:tstep:tf;

R = 1.*eye(4);
Q = 0.000001.*eye(4);
P0 = zeros(4,4);

out = sim('Navigation_Sim');
q_real = out.Quaternion.signals.values;
q_estimated = out.Quaternion_hat.signals.values;

figure()
for i=1:4
    subplot(2,2,i)
    plot(t,q_real(:,i),t,q_estimated(:,i))
end
legend('Real','Estimated')
