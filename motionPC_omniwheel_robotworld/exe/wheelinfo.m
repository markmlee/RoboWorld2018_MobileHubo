clear
clc
close all

file = load('data.txt');

InitRWH = file(:,1);
InitLWH = file(:,2);
InitBWH = file(:,3);

CurRefRWH = file(:,4);
CurRefLWH = file(:,5);
CurRefBWH = file(:,6);

dCurRefRWH = file(:,7);
dCurRefLWH = file(:,8);
dCurRefBWH = file(:,9);

GoalRWH = file(:,10);
GoalLWH = file(:,11);
GoalBWH = file(:,12);

InRWHm = file(:,13);
InLWHm = file(:,14);
InBWHm = file(:,15);

svRWH = file(:,19);
svLWH = file(:,20);
svBWH = file(:,21);
wvRWH = file(:,22);
wvLWH = file(:,23);
wvBWH = file(:,24);
MoveRWH = file(:,28);
MoveLWH = file(:,29);
MoveBWH = file(:,30);

figure
hold on;
plot(CurRefRWH);
plot(CurRefLWH);
plot(CurRefBWH);
legend('curRWH','curLWH','curBWH');

figure
hold on;
plot(dCurRefRWH);
plot(dCurRefLWH);
plot(dCurRefBWH);
plot(svRWH);
plot(wvRWH);
legend('dcurRWH','dcurLWH','dcurBWH','svRWH','wvRWH');

figure
hold on;
plot(MoveRWH);
plot(MoveLWH);
plot(MoveBWH);
plot(InRWHm);
plot(InLWHm);
plot(InBWHm);
legend('mRWH','mLWH','mBWH','InR','InL','InB');
