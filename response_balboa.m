%Parameters from Balboa
mc = 0.316;                         %body part mass [kg]
mw = 2 * 0.021 ;                    %wheel(*2) mass [kg]
L  = 23.0 * 1e-3;                   %position of COM [m]

Ic  = 444.43 * 1e-6;                 %inertia of body part [kg*m^2]
Iw = 2 * 26.89 * 1e-6;              %inertia of wheel [kg*m^2]

Br=0.0;                            %rolling damping ratio [N*m/(rad/s)]
Bm=0.0;                            %bearing damping ratio [N*m/(rad/s)]

R=40*10^(-3);                       %radius of wheel [m]
g=9.81;                             %gravity [m/s^2]

    
%Weighting matrices
E=[Iw+(mw+mc)*R*R mc*R*L; mc*R*L mc*L*L+Ic];  %for d^2/dt^2 (phi and theta)
F=[Br+Bm -Bm; -Bm Bm];                   %for d/dt (phi and theta)
G=[0; -mc*g*L];                           %for theta
H=[1; 0];                               %for input torque

%state-space representation of the system
%state variable: phi, theta, d(phi)/dt, d(theta)/dt
A=[0 0 1 0; 0 0 0 1; [0; 0] -E\G -E\F];        %system matrix
B=[0; 0; E\H];                                      %input matrix
C=[1 0 0 0; 0 1 0 0];                                    %output matrix
D=0;
sys1=ss(A,B,C,D);
E = eig(A);

G1=tf(sys1);                           %transfer function of sys1
G1zp=zpk(sys1);                        %Gain/pole/zero representation of sys1

%LQR controller design                                  
Q = [1,0,0,0;...                            %weighting matrix Q
     0,1,0,0;...
     0,0,1,0;...
     0,0,0,1];
R = 1;                                      %weighting matrix R
[K, S, P] = lqr(A, B, Q, R)                         %gain matrix

sys=ss(A,B,C,D);
initial(sys, [0; 0.17; 0; 0])

sysCL=ss(A-B*K,B,C,D);
initial(sysCL, [0; 0.17; 0; 0])

poles_bessel1 = [-0.657+0.830i, -0.657-0.830i, -0.905+0.271i, -0.905-0.271i];
K_bessel1 = place(A,B,poles_bessel1);
eig(A-B*K_bessel1);



poles_bessel2 = [-4.016+5.072i, -4.016-5.072i, -5.528+1.655i, -5.528-1.655i];
K_bessel2 = place(A,B,poles_bessel2);
eig(A-B*K_bessel2);

poles_ITAE1 = [-0.424+1.263i, -0.424-1.263i, -0.626+0.414i, -0.626-0.414i];
K_ITAE1 = place(A,B,poles_ITAE1);
eig(A-B*K_ITAE1);



poles_ITAE2 = [-4.236+12.617i, -4.236-12.617i, -6.254+4.139i, -6.254-4.139i];
K_ITAE2 = place(A,B,poles_ITAE2);
eig(A-B*K_ITAE2);

poles_test = [-0.4016+0.5072i, -0.4016-0.5072i, -0.5528+0.1655i, -0.5528-0.1655i];
K_test = place(A,B,poles_test)
eig(A-B*K_test)






sys1_lqr=ss((A - B*K), B, C, D);            %close-loop system

%E = eig(A - B*K)
factor = 2.120288;
Knew = [-2.120288/factor -56.783782/factor -0.958486/factor -7.949175/factor];
%Enew = eig(A - B*Knew)

x0 = [0; 0.17; 0; 0];
t = 0:0.1:5;
%initial(sys1_lqr, x0, t);             %free response  



%controllability and observability check for sys1
Cont=[B A*B A*A*B A*A*A*B];
rank(Cont);
Obs=[C; C*A; C*A*A; C*A*A*A];
rank(Obs);

