clear;
clc;

mdl_puma560

T0=p560.fkine(qn)

dq=1e-6;

Tp1=p560.fkine(qn+[dq 0 0 0 0 0])

%find the drevative of T with respect to theta 01
%to understand the effect of theta 01 on the orientation of the end
%effector
dTdq1=(Tp1-T0)/dq
transl(dTdq1)
pause
