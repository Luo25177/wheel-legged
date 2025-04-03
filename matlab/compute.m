clc
clear
%% 开始解算
syms x(t) theta(t) phi(t)
syms Tw Tb Pw Pb Nw Nb
syms x_ddot theta_ddot phi_ddot x_dot theta_dot phi_dot
syms L Il Lw Lb ml
mb = 0.972000;
Ib_y = 0.002697;
g = 9.81;
l = 0;

Il = 0.000808;
Lw = 0.1;
Lb = 0.1;
ml = 0.240000;
L = 0.2;

mw = 0.471239;
R = 0.05;
Iw = 0.000589;

func1 = [ml * diff(diff(x + Lw * sin(theta), t), t) == Nw - Nb;
    ml * diff(diff(Lw * cos(theta), t), t) == Pw - Pb - ml * g;
    mb * diff(diff(x + L * sin(theta) - l * sin(phi), t), t) == Nb;
    mb * diff(diff(L * cos(theta) + l * cos(phi), t), t) == Pb - mb * g;
    ];

[Pw, Pb, Nw, Nb] = solve(func1, [Pw, Pb, Nw, Nb]);

func2 = [diff(diff(x, t), t) == (Tw * R - Nw * R * R) / (Iw + mw * R * R);
    Il * diff(diff(theta, t), t) == Tb - Tw + (Pb * Lb + Pw * Lw) * sin(theta) - (Nb * Lb + Nw * Lw) * cos(theta);
    Ib_y * diff(diff(phi, t), t) == Tb + Nb * l * cos(phi) + Pb * l * sin(phi);];

func2 = subs(func2, ...
    [diff(diff(x, t), t), diff(diff(theta, t), t), diff(diff(phi, t), t), diff(x, t), diff(theta, t), diff(phi, t)], ...
    [x_ddot, theta_ddot, phi_ddot, x_dot, theta_dot, phi_dot]);

[x_ddot, theta_ddot, phi_ddot] = solve(func2, [x_ddot theta_ddot phi_ddot]);

X = [theta(t); theta_dot; x(t); x_dot; phi(t); phi_dot];
u = [Tw; Tb];
X_dot = [theta_dot; theta_ddot; x_dot; x_ddot; phi_dot; phi_ddot];
A = jacobian(X_dot, X);
B = jacobian(X_dot, u);
A = subs(A, [x_dot, theta(t), theta_dot, phi(t), phi_dot, Tw, Tb], zeros(1,7));
B = subs(B, [x_dot, theta(t), theta_dot, phi(t), phi_dot, Tw, Tb], zeros(1,7));

valA = double(A);
valB = double(B);

if(rank(ctrb(valA, valB)) == size(valA, 1))
    disp('系统可控')
else
    disp('系统不可控')
    K = 0;
    return
end

C = eye(6);
D = zeros(6,2);
Q = diag([1 1 500 100 5000 1]);
R = diag([1 0.25]);
sys = ss(valA, valB, C, D);
KLQR = lqr(sys, Q, R);
