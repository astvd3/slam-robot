function [r,a,covM]=compute_line_params(P,Q,sigp,sigt)

n=length(P);

%Compute N,D, Eq. (107) & (108)
N1=0;
N2=0;
D1=0;
D2=0;
for i=1:n
    for j=1:n
        N1=N1+P(i)*P(j)*cos(Q(i))*sin(Q(j));
        D1=D1+P(i)*P(j)*cos(Q(i)+Q(j));
    end
    N2=N2+P(i)^2*sin(2*Q(i));
    D2=D2+P(i)^2*cos(2*Q(i));
end

N=2*N1/n-N2;
D=D1/n-D2;

%Compute alpha, Eq. (106)
a=0.5*atan2(N,D);

%compute r, Eq. (71)
r=P*cos(Q'-a)/n;

if r<0
    r=-r;
    a=atan2(sin(a+pi),cos(a+pi));
end

%compute xbar, ybar, Eq. (111)
xbar=(1/n)*P*cos(Q');
ybar=(1/n)*P*sin(Q');

da_dP=zeros(1,n);
dr_dP=da_dP;
for i=1:n
    %compute da/dP vector, Eq. (130)
    da_dP(i)=(N*(xbar*cos(Q(i))-ybar*sin(Q(i))-P(i)*cos(2*Q(i)))...
        -D*(xbar*sin(Q(i))+ybar*cos(Q(i))-P(i)*sin(2*Q(i))))/(D^2+N^2);
    
    %compute dr/dP vector, Eq. (137)
    dr_dP(i)=cos(Q(i)-a)/n+da_dP(i)*(ybar*cos(a)-xbar*sin(a));
end

%calculate sigma_A^2, Eq. (91)
sigA2=da_dP*da_dP'*sigp;

%calculate sigma_R^2, Eq. (92)
sigR2=dr_dP*dr_dP'*sigp;

%calculate sigma_AR, Eq. (141)
sigAR=da_dP*dr_dP'*sigp;

%Assemble final covariance matrix
covM=[sigA2 sigAR;sigAR sigR2];
