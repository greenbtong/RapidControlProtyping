clear all;
[v,T,vT]=xlsread('steadystate.xlsx'); 

t=v(:,1);
y=v(:,2);

figure
plot(t,y)
xlabel('Fan Control Signal')
ylabel('Filtered Thrust Measurement (N)')
title('Steady-State characteristics')
%%
close all
clear
clc

V_ab = [0 0.5 1 1.5 2 2.5 3 3.5 4 4.5 5 5.5 6 6.5 7 7.5 8 8.5 9 9.5 10];          % Fan voltage input (V)
Fss_ab = [0 0 0 0.0001 0.0001 0.0001 0.0017 0.0028 0.0042 0.0059 0.0076 0.0096 0.012 0.0142 0.0164 0.0192 0.0216 0.0242 0.0269 0.0292 0.0319];          % Steady-state fan thrust output (N)
Ts = .1E-5;
p = 1;
kkk = 1;
for i = V_ab
    F(p) = 1/(Ts+1)* Fss_ab(p)* 9.81/1000/i;
    Fs(p) = Fss_ab(p) * (1/i - 1/(i+1/Ts));
    Ft(p) = Fss_ab(p) *(1-exp(-1/Ts*i));
    p = p +1;
    
end
plot(V_ab, Ft)
%plot(V_ab, Fss_ab)
%F(20)

FanDynData = load('FanDynData.txt');
TimeArray = FanDynData(1,:);
Excitation = FanDynData(2,:);
Response = FanDynData(3,:);
figure(1)
plotyy(TimeArray,Excitation,TimeArray,Response)
%plot(TimeArray, Response)
hold
plot(V_ab,Ft)
grid
xlabel('Time (s)')
ylabel('Force (N)')
xlim([0,10])
legend('Control Signal Excitation','Force (g)')

%%
tt = 1.10:0.1:10;
plot(TimeArray, Response)
xlabel('Time (s)')
ylabel('Force (N)')
xlim([0,10])
xlabel('Time (s)')
ylabel('Force (N)')
title('Fan 12V step response')

hold on
plot(tt, 0.0319 * (1-exp((1.10-tt)/0.65)))
hold off


