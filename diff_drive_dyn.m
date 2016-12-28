function V_next=diff_drive_dyn(robot_params,V_in,tau,Ts)
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

%Robot Linear and angular velocity
v=V_in(1);
w=V_in(2);

%Wheel Speeds
phDot=[ 1/r      b/r;
       1/r     -b/r]*V_in;
phRdot=phDot(1);
phLdot=phDot(2);

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

%Input Transformation
B=[1/r  1/r;
   b/r -b/r];

%Velocity at time k+1
V_next=V_in+Ts*M^(-1)*(B*tau-td-G-F-Vm*V_in);