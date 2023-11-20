%Parameters from Balboa
mr = 0.316;                         %body part mass [kg]
mw = 2 * 0.021 ;                    %wheel(*2) mass [kg]
L  = 23.0 * 1e-3;                   %position of COM [m]

I  = 444.43 * 1e-6;                 %inertia of body part [kg*m^2]
Iw = 2 * 26.89 * 1e-6;              %inertia of wheel [kg*m^2]

Br=0.01;                            %rolling damping ratio [N*m/(rad/s)]
Bm=0.01;                            %bearing damping ratio [N*m/(rad/s)]

R=40*10^(-3);                       %radius of wheel [m]
g=9.81;                             %gravity [m/s^2]

    
%Weighting matrices
E=[Iw+(mw+mr)*R*R mr*R*L; mr*R*L mr*L*L+I];  %for d^2/dt^2 (phi and theta)
F=[Br+Bm -Bm; -Bm Bm];                   %for d/dt (phi and theta)
G=[0; -mr*g*L];                           %for theta
H=[1; -1];                               %for input torque

%state-space representation of the system
%state variable: phi, theta, d(phi)/dt, d(theta)/dt
A=[0 0 1 0; 0 0 0 1; [0; 0] -E\G -E\F];        %system matrix
B=[0; 0; E\H];                                      %input matrix
C=[R 0 0 0; 0 1 0 0];                                    %output matrix
D=0;
sys1=ss(A,B,C,D);


G1=tf(sys1);                           %transfer function of sys1
G1zp=zpk(sys1);                        %Gain/pole/zero representation of sys1


%LQR controller design
xweight=eye(4);                                 %weighting matrix Q
uweight=1;                                      %weighting matrix R
K=-lqr(A,B,xweight,uweight);                    %gain matrix
sys1_lqr=ss(A+B*K,B,C,D);                       %close-loop system
initial(sys1_lqr, [0; 0.17; 0; 0]);             %free response  
%controllability and observability check for sys1
Cont=[B A*B A*A*B A*A*A*B];
rank(Cont);
Obs=[C; C*A; C*A*A; C*A*A*A];
rank(Obs);

