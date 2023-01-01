clear
clc

%system
A = [0 -4.5 0; 0 -0.025 0.000013;0 0 -5.54/60 ];
B = [0 0 1/12]';
C = [1 0 0];
D = 0;

Gp = ss(A,B,C,D);
[num,den] = ss2tf(A,B,C,D);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Question2%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%controller P + I/s + D Ns/(s+N)
P = -12.61;
I = -0.0456;
D = -381.1;
N_ = 0.07307;

Gc = tf([P+D*N_ P*N_+I I*N_],[1 N_ 0]);
stepinfo(feedback(series(Gp,Gc),1))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Question3%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%discretize controller
Ts = [ 0.1 0.9 100];
N_matched=[];
D_matched=[];

N_bilinear=[];
D_bilinear=[];

for i=1:3
    Gd = c2d(Gc, Ts(i), 'matched');
    [n, d]=tfdata(Gd);
    N_matched=[N_matched;n{1}];
    D_matched=[D_matched;d{1}];
    
    Gd = c2d(Gc, Ts(i), 'tustin');
    [n, d]=tfdata(Gd);
    N_bilinear=[N_bilinear;n{1}];
    D_bilinear=[D_bilinear;d{1}];
end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Question7%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%discretize system
Gd = c2d(tf(num,den),9,'zoh');
[Nd, Dd]=tfdata(Gd);
zpk(Gd)

%%deadbeat computations
syms f1 f2 f3

a = 0.2032;
g = simplify(convolve([f3/2.905 f1], conv([-0.7985 1],[-0.4356 1])));
g = g(end:-1:1);

b0 = g(1);
b1 = g(2) - a*b0;
b2 = g(3) - a*b1;

eqns = [ f1+f2+f3==1 , 2.905^2*f1-2.905*f2+f3==0 , a*b2==g(4)];
vars = [f1 f2 f3];
f=solve(eqns,vars);

f1 = double(f.f1);
f2 = double(f.f2);
f3 = double(f.f3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Question9%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%discretize in state space
G = expm(9*A);
H = integral(@(t)(expm(t*A)*B),0,9,'ArrayValued',1);

%controllability and observability
M=[H G*H G*G*H];
rank_M = rank(M);
N=[C;C*G;C*G*G];
rank_N = rank(N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Question10%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%dead-beat state feedback controller
a = charpoly(G);
a = a(2:end);
W = [a(2) a(1) 1; a(1) 1 0; 1 0 0 ];
T = M*W;

G_ = T^-1*G*T;
H_ = T^-1*H;
K_ = G_(3,:);
K = K_*T^-1;

k_follow= (C*(eye(3)-G+H*K)^-1*H)^-1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Question11%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%full-order state observer
T = (W*N)^-1;
G_ = T^-1*G*T;
H_ = T^-1*H;
C_ = C*T;
L_ = G_(:,3);
L = T*L_;

E_ = G_-L_ *C_;
E = T*E_*T^-1;



