clc
clear
%% 拟合质心位置，转动惯量和杆长的函数
I = zeros(1, 116);
L = zeros(1, 116);
Lw = zeros(1, 116);
Lb = zeros(1, 116);

for angle4 = -15 : 1 : 100
    [ml, Il, L_, Lw_, Lb_] = GetLegBaryCenter(180 - angle4, angle4, 0);
    I(angle4 + 16) = Il;
    L(angle4 + 16) = L_;
    Lw(angle4 + 16) = Lw_;
    Lb(angle4 + 16) = Lb_;
end

KI = polyfit(L, I, 1);
valKI = polyval(KI,L);
figure(1);hold on;plot(L,I,'r*',L,valKI,'b-.');

KLw = polyfit(L, Lw, 1);
valKLw = polyval(KLw,L);
figure(1);hold on;plot(L,Lw,'r*',L,valKLw,'b-.');


KLb = polyfit(L, Lb, 1);
valKLb = polyval(KLb,L);
figure(1);hold on;plot(L,Lb,'r*',L,valKLb,'b-.');

%% 开始解算
syms x(t) theta(t) phi(t)
syms Tw Tb Pw Pb Nw Nb
syms x_ddot theta_ddot phi_ddot x_dot theta_dot phi_dot
syms L
mw = 1.267245 * 2;
R = 0.1;
Iw = 0.00379267 * 2;
mb = 5.4940204;
Ib_y = 0.05026821;
g = 9.81;
Ic_z = 0.37248874;
R_l = 0.482000001;
l = -0.01994485;
Il = (KI(1, 1) * L + KI(1, 2)) * 2;
Lw = KLw(1, 1) * L + KLw(1, 2);
Lb = KLb(1, 1) * L + KLb(1, 2);
ml = ml * 2;

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

%% 设置拟合次数和腿长，开始拟合
numsize = 29;
minleglen = 0.120;
maxleglen = 0.400;

K_vals = zeros(numsize, 2, 6);
A_vals = zeros(numsize, 6, 6);
B_vals = zeros(numsize, 6, 2);
L_ranges = linspace(minleglen, maxleglen, numsize);

for i = 1 : 1 : numsize
valL = L_ranges(i);

valA = subs(A, L, valL);
valB = subs(B, L, valL);
valA = double(valA);
valB = double(valB);

if(rank(ctrb(valA, valB)) == size(valA, 1))
    disp('系统可控')
else
    disp('系统不可控')
    K = 0;
    return
end
disp(i)

C = eye(6);
D = zeros(6,2);
Q = diag([10.0000 1.0000 50.0000 30.0000 100.0000 10.0000]);
R = diag([1 0.25]);
sys = ss(valA, valB, C, D);
KLQR = lqr(sys, Q, R);%得到反馈增益矩阵
K_vals(i, :, :) = KLQR;
A_vals(i, :, :) = valA;
B_vals(i, :, :) = valB;
end

%% LQR 拟合:比较倾向于使用Curve Fitting Toolbox，简单好用
K11 = K_vals(:, 1, 1);
K12 = K_vals(:, 1, 2);
K13 = K_vals(:, 1, 3);
K14 = K_vals(:, 1, 4);
K15 = K_vals(:, 1, 5);
K16 = K_vals(:, 1, 6);

K21 = K_vals(:, 2, 1);
K22 = K_vals(:, 2, 2);
K23 = K_vals(:, 2, 3);
K24 = K_vals(:, 2, 4);
K25 = K_vals(:, 2, 5);
K26 = K_vals(:, 2, 6);

A11 = A_vals(:, 1, 1);
A12 = A_vals(:, 1, 2);
A13 = A_vals(:, 1, 3);
A14 = A_vals(:, 1, 4);
A15 = A_vals(:, 1, 5);
A16 = A_vals(:, 1, 6);

A21 = A_vals(:, 2, 1);
A22 = A_vals(:, 2, 2);
A23 = A_vals(:, 2, 3);
A24 = A_vals(:, 2, 4);
A25 = A_vals(:, 2, 5);
A26 = A_vals(:, 2, 6);

A31 = A_vals(:, 3, 1);
A32 = A_vals(:, 3, 2);
A33 = A_vals(:, 3, 3);
A34 = A_vals(:, 3, 4);
A35 = A_vals(:, 3, 5);
A36 = A_vals(:, 3, 6);

A41 = A_vals(:, 4, 1);
A42 = A_vals(:, 4, 2);
A43 = A_vals(:, 4, 3);
A44 = A_vals(:, 4, 4);
A45 = A_vals(:, 4, 5);
A46 = A_vals(:, 4, 6);

A51 = A_vals(:, 5, 1);
A52 = A_vals(:, 5, 2);
A53 = A_vals(:, 5, 3);
A54 = A_vals(:, 5, 4);
A55 = A_vals(:, 5, 5);
A56 = A_vals(:, 5, 6);

A61 = A_vals(:, 6, 1);
A62 = A_vals(:, 6, 2);
A63 = A_vals(:, 6, 3);
A64 = A_vals(:, 6, 4);
A65 = A_vals(:, 6, 5);
A66 = A_vals(:, 6, 6);

B11 = B_vals(:, 1, 1);
B12 = B_vals(:, 1, 2);

B21 = B_vals(:, 2, 1);
B22 = B_vals(:, 2, 2);

B31 = B_vals(:, 3, 1);
B32 = B_vals(:, 3, 2);

B41 = B_vals(:, 4, 1);
B42 = B_vals(:, 4, 2);

B51 = B_vals(:, 5, 1);
B52 = B_vals(:, 5, 2);

B61 = B_vals(:, 6, 1);
B62 = B_vals(:, 6, 2);
