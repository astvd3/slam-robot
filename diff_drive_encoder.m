function [est_robot_pos_out,estV_out,sigP_out]=diff_drive_encoder(V_in,est_robot_pos_in,sigP_in,Ts)

r=0.25;      %wheel radius
b=0.8;       %wheel base
R=[r/2 r/2;
    r/b -r/b];

kR=.01;
kL=.01;

thhat=est_robot_pos_in(3);

wRL=inv(R)*V_in;  % calculate wheel speeds

wR=wRL(1);
wL=wRL(2);

nR=sqrt(kR*abs(Ts*wR));  %inject max 1-sigma noise value
nL=-sqrt(kL*abs(Ts*wL));

enR_error=1;
enL_error=1;

%Inject Encoder Errors
if randi(100)>95
    enR_error=randi(1);
end

if randi(100)>95
    enL_error=randi(1);
end

wRhat=wR+enR_error*nR;
wLhat=wL+enL_error*nL;

estV_out=R*[wRhat;wLhat];
vhat=estV_out(1);
what=estV_out(2);

S=[cos(thhat+Ts*what/2) 0;
    sin(thhat+Ts*what/2) 0;
    0                     1];

%encoder estimates
est_robot_pos_out=est_robot_pos_in+Ts*S*estV_out;

%Eq 5.8
sig_d=[kR*abs(Ts*wRhat*r) 0;0 kL*abs(Ts*wLhat*r)];

%Eq 5.10
Fp=[1 0 -Ts*vhat*sin(thhat+Ts*what/2);
    0 1  Ts*vhat*cos(thhat+Ts*what/2);
    0 0       1];

%Eq 5.11
Fdrl=[0.5*cos(thhat+Ts*what/2)-Ts*vhat*sin(thhat+Ts*what/2)/(2*b) 0.5*cos(thhat+Ts*what/2)+Ts*vhat*sin(thhat+Ts*what/2)/(2*b);
    0.5*sin(thhat+Ts*what/2)+Ts*vhat*cos(thhat+Ts*what/2)/(2*b) 0.5*sin(thhat+Ts*what/2)-Ts*vhat*cos(thhat+Ts*what/2)/(2*b);
    1/b                                                                   -1/b];

%Eq 5.9
sigP_out=Fp*sigP_in*Fp'+Fdrl*sig_d*Fdrl';