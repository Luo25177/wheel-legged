clc
clear
%% 开始解算
syms x(t) theta(t) phi(t)
syms Tw Tb Pw Pb Nw Nb
syms x_ddot theta_ddot phi_ddot x_dot theta_dot phi_dot
syms L Il Lw Lb ml
mw = 0.77115000 * 2;
R = 0.1;
Iw = 0.00379267 * 2;

mb = 5.4940204;
Ib_y = 0.05019911;
g = 9.81;
Ic_z = 0.37248874;
R_l = 0.482000001;
l = -0.02011323;

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
minangle4 = -15;
maxangle4 = 79;
steps = 0.2;
numsize = (maxangle4 - minangle4) / steps + 1;

K_vals = zeros(numsize, 2, 6);
L_vals = zeros(numsize, 1);

angle1_vals = zeros(numsize, 1);
angle4_vals = zeros(numsize, 1);

a = 1;
for angle4 = minangle4 : steps : maxangle4
    angle1 = 180 - angle4;
    [valml, valIl, valL, valLw, valLb] = GetLegBaryCenter(angle1, angle4, 0);
    valA = subs(A, [ml, Il, L, Lw, Lb], [2 * valml, 2 * valIl, valL, valLw, valLb]);
    valB = subs(B, [ml, Il, L, Lw, Lb], [2 * valml, 2 * valIl, valL, valLw, valLb]);
    valA = double(valA);
    valB = double(valB);
    
    if(rank(ctrb(valA, valB)) == size(valA, 1))
        disp('系统可控')
    else
        disp('系统不可控')
        K = 0;
        return
    end
    
    C = eye(6);
    D = zeros(6,2);
    Q = diag([1 1 200 80 500 1]);
    R = diag([2 0.5]);
    sys = ss(valA, valB, C, D);
    KLQR = lqr(sys, Q, R);%得到反馈增益矩阵
    K_vals(a, :, :) = KLQR;
    L_vals(a) = valL;   
    a = a + 1;
end

K_coefficient = [];
for i = 1:1:2
    for j = 1 : 1 : 6
    column_K = K_vals(:,i, j);
    K_coefficient = [K_coefficient; polyfit(L_vals, column_K, 3)];%polyfit(x, y, n)多项式拟合，注意参数别写反了
    end
end
disp(K_coefficient)

%% LQR 拟合:比较倾向于使用 Curve Fitting Toolbox ，简单好用
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
