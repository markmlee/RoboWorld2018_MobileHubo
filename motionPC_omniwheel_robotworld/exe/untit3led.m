file = load('data.txt');

NoLiftT = [1:1:1600];
LiftT = [6200:1:7930];

RFx = file(:,40);
RFy = file(:,41);
RFz = file(:,42);


LFx = file(:,43);
LFy = file(:,44);
LFz = file(:,45);

RHAND = file(:,46);
LHAND = file(:,47);

RHx = file(:,48);
RHy = file(:,49);
RHz = file(:,50);

Command = file(:,62);
figure
hold on;
plot(RHx(NoLiftT),RHz(NoLiftT),'r');
plot(RHx(LiftT),RHz(LiftT),'b');

figure
hold on;
plot(RFz);
plot(RFx);
plot(RFy);
legend('z','x','y');

figure
hold on;
plot(RHAND);
plot(Command);