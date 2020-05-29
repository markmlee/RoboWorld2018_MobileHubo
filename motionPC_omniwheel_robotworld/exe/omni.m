clear
clc
close all

file = load('data.txt');

inX = file(:,34);
inY = file(:,35);
inTheta = file(:,36);


pX = file(:,37);
pY = file(:,38);
pTheta = file(:,39);


vX = file(:,40);
vY = file(:,41);
vTheta = file(:,42);

v1 = file(:,43);
v2 = file(:,44);
v3 = file(:,45);

p1 = file(:,46);
p2 = file(:,47);
p3 = file(:,48);


figure
hold on;
plot(inX);
plot(pX);
plot(vX);
legend('inX','p','V');


figure
hold on;
plot(inY);
plot(pY);
plot(vY);
legend('inY','p','V');


figure
hold on;
plot(inTheta);
plot(pTheta);
plot(vTheta);
legend('inTheta','p','V');

figure
hold on;
plot(v1);
plot(v2);
plot(v3);
legend('1','2','3');


figure
hold on;
plot(p1);
plot(p2);
plot(p3);
legend('p1','2','3');