function tau=robot_backstepping_control(robot_params,V,Vd,Vd_dot,x)
%Robot parameters
mT=robot_params.mT;  %Total mass
mW=robot_params.mW;   %Mass of 1 wheel
r=robot_params.r; %Wheel radius
b=robot_params.b;  %half the robot width
d=robot_params.d;  %CG offset from rear axle
Iyy=robot_params.Iyy;  %Wheel moment of Iertia
IT=robot_params.IT;   %Platform total moment of inertia
Fv=robot_params.Fv;  %Coefficient of Fiscous friction
Fd=robot_params.Fd;  %Coefficient of Colomb friction
t_max=robot_params.tau_max; %Max Motor Torque

k1=10;
k2=5;
k3=4;


%Robot Linear and angular velocity
v=V(1);
w=V(2);

%%
%Robot States
xc=x(1);
yc=x(2);
th=x(3);

v=x(4);
w=x(5);
V=[v;w];

%%
%Reference Robot
xd=x(6);
yd=x(7);
thd=x(8);

%Reference Velocity
vd=x(9);
wd=x(10);

vddot=x(11);
wddot=x(12);
%%
e=[cos(th) sin(th) 0;
  -sin(th)  cos(th) 0;
   0          0    1]*[xd-xc; yd-yc; thd-th];

Vd=[vd;wd];
Vddot=[vddot;wddot];

Vc=[vd*cos(e(3))+k1*e(1); wd+k2*vd*e(2)+k3*vd*sin(e(3))];

edot=[0 w 0;-w 0 0;0 0 0]*e+[vd*cos(e(3));vd*sin(e(3));wd]-[1 0;0 0;0 1]*V;

Vcdot=[vddot*cos(e(3))+vd*(-sin(e(3)))*edot(3)+k1*edot(1);
    wddot+k2*vddot*e(2)+k2*vd*edot(2)+k3*vddot*sin(e(3))+k3*vd*cos(e(3))*edot(3)];

%Wheel Speeds
phDot=[ 1/r      b/r;
       1/r     -b/r]*V;
phRdot=phDot(1);
phLdot=phDot(2);

B=[1/r  1/r;
   b/r -b/r];

%dynamics
%Mass matrix
M=[mT+2*Iyy/(r^2)                   0;
    0           mT*d^2+IT+2*Iyy*(b^2)/(r^2)-4*mW*d^2];

%Centripital and Coriolis
Vm=[0       -d*w*(mT-2*mW);
    d*w*(mT-2*mW)   0];

%Friction
F=(1/r)*[Fv*(phRdot+phLdot)+Fd*(sign(phRdot)+sign(phLdot));
      b*(Fv*(phRdot-phLdot)+Fd*(sign(phRdot)-sign(phLdot)))];

%Gravity
G=[0;0];

%Disturbance
td=[0;0];

%Control Gain
K=[20 0;0 20];

hc=[-e(1);sin(e(3))/k2];
f=-M^(-1)*(Vm*V+F+G);
%Basic Proportional Controller
%tau=B^(-1)*(-K*(V-Vd));

%Put your code here to calculate the values for tau (torque to the wheel
%motors) using the backstepping technique
tau=B^(-1)*M*(-K*(V-Vc)+Vcdot+hc-f);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Enforce Max Torque
[tau_max,I]=max(abs(tau));
if tau_max>t_max
    tau_temp=t_max*sign(tau(I(1)));
    switch I(1)
        case 1 %tau 1 is max
            tau(2)=tau(2)*abs(tau_temp/tau(1));
            tau(1)=tau_temp;
        case 2 %tau 2 is max
            tau(1)=tau(1)*abs(tau_temp/tau(2));
            tau(2)=tau_temp;
        otherwise
    end
end
    
    
    
    